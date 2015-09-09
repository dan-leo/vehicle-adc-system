/**
 * 	vehicleMon.c:
 *
 *  Requires:
 *  	4D Systems Visi-Genie touchscreen
 *		18-bit 8-channel ABElectronics Analogue-Digital-Converter
 *		Any Raspberry Pi
 *
 *		git clone https://github.com/4dsystems/ViSi-Genie-RaspPi-Library.git
 *		apt-get install libi2c-dev
 *
 *  Setup parameters via touchscreen.
 *  Parameters include: voltage divider resistance constant, min and max voltage before alarm condition, dis/arming alarm,
 *      voltage offset.
 *	Voltage divider resistance constant can be calculated automatically if reference voltage is present.
 *  Monitor voltages of a vehicle (alternator, battery, boost intake pressure) via i2c from ADC.
 *  Print voltages to touchscreen.
 *
 *
 *  Daniel Robinson, 9 April 2015
 *  Daniel Robinson, 10 June 2015
 ***********************************************************************
 */

#define TRUE 1
#define FALSE 0
#define display_length 16
#define channels 8
#define line_length 255

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <math.h>

#include <pthread.h>

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <wiringPi.h>
#include <softPwm.h>

#include <geniePi.h>

#include "adcpiv3.h"


int current_form, previous_form, pre_previous_form;
int errorCondition;
int current_slider = -1;
int last_edit_button;
int volume = 10;

int slider_values[channels];
int rocker_values[channels];
int armed[channels];
int alarm_activated[channels];

double true_voltage[channels];
double modified_voltage[channels];
double gradient[channels];
double offset[channels];
double max[channels];
double min[channels];
double ref_volt_1[channels] = {0};
double ref_volt_2[channels] = { [0 ... (channels - 1)] = 12};
double true_volt_1[channels];
double true_volt_2[channels];
double alarm_max[channels];
double alarm_min[channels];

const double max_volt = 2.048;
const double min_volt = -2.048;

char display[display_length];
char numberString[display_length];
char *data_file = "data.txt";

FILE *fp;

enum op_form 
{
	HOME,
	SCOPE,
	CALIBRATE,
	NUMPAD,
	CONFIRMATION,
	AUTO,
	SETTINGS,
	SETUP_ALARM,
	ALARM
};

enum op_channel
{
	CH_1 = 9,
	CH_2 = 8,
	CH_3 = 10,
	CH_4 = 7,
	CH_5 = 6,
	CH_6 = 2,
	CH_7 = 1,
	CH_8 = 0,
	ROCKER_CH_1 = 15,
	ROCKER_CH_2 = 13,
	ROCKER_CH_3 = 14,
	ROCKER_CH_4 = 16,
	ROCKER_CH_5 = 17,
	ROCKER_CH_6 = 18,
	ROCKER_CH_7 = 19,
	ROCKER_CH_8 = 20
};

int slider[channels] = {CH_1, CH_2, CH_3, CH_4, CH_5, CH_6, CH_7, CH_8};
int rocker[channels] = {ROCKER_CH_1, ROCKER_CH_2, ROCKER_CH_3, ROCKER_CH_4, ROCKER_CH_5, ROCKER_CH_6, ROCKER_CH_7, ROCKER_CH_8};

enum win_button
{
	BUT_GRAD = 2,
	BUT_OFFS = 3,
	BUT_YES = 5,
	BUT_NO = 6,
	BUT_KB_BACK = 8,
	BUT_CH_1 = 9,
	BUT_SAVE_1 = 11,
	BUT_CH_2 = 12,
	BUT_SAVE_2 = 13,
	BUT_MAX = 18,
	BUT_MIN = 17,
	BUT_ALARM_RESET = 20,
	BUT_ALARM_MAX = 21,
	BUT_ALARM_MIN = 22,
	BUT_ALARM_DISARM = 23,
	BUT_ALARM_ARM = 24,
	BUT__ALARM = 25,
	BUT__ALARM_DISARM_ALL = 27,
	BUT_REBOOT = 30,
	BUT_SHUTDOWN = 31
};

enum button_4D
{
	BUT_4D_AUTO = 11,
	BUT_4D_RESET = 12
};

enum keys
{
	KB_BACKSPACE = 176,
	KB_SIGN_CHANGE = 107,
	KB_SAVE = 13,
	KB_DOT = 110
};


void updateDisplay (double val, int index);
int setup(void);
static void *adc_read_loop (void *data);
void handleGenieEvent (struct genieReplyStruct *reply);
void updateForm(int form);
void updateNumpadDisplay (void);
void updateGraphFormula (void);
void updateAutoScreen (void);
void updateAlarm (void);
void processKey (int key);
int isNumeric(char *str);
void reset(void);
void reset_alarm_min_max(void);
void save_to_file(void);
void updateRange(void);

/*
 *********************************************************************************
 * main:
 *  Run the touchscreen adc
 *********************************************************************************
 */

int main(int argc, char **argv) {
	int i, j, channel;
	pthread_t myThread;
	struct genieReplyStruct reply;


	setup();

	// setup multiplier based on input voltage range and divisor
	// removed 2.4705882 constant in place of 1 
	varMultiplier = (1 / varDivisior) / 1000;

	// possibly useful code
	if (argc > 1) channel = atoi(argv[1]);
	if (channel < 1 | channel > 8) channel = 1;

	// start adc read thread
	(void)pthread_create (&myThread, NULL, adc_read_loop, NULL);

	// touchscreen event loop
	for (;;)
	{
		while (genieReplyAvail())
		{
			genieGetReply    (&reply);
			handleGenieEvent (&reply);
		}
		usleep (10000); // 10mS - Don't hog the CPU in-case anything else is happening...
	}
	return 0;
}

int setup(void)
{
	int i;
	char *line;
	const char t[2] = ",";
	char *token;

	// Genie display setup
	// Using the Raspberry Pi's on-board serial port.
	if (genieSetup ("/dev/ttyAMA0", 115200) < 0)
	{
		fprintf (stderr, "rgb: Can't initialise Genie Display: %s\n", strerror (errno));
		return 1;
	}

	// volume
	genieWriteObj(GENIE_OBJ_SOUND, 1, volume);

	// Select form 0, Home
	genieWriteObj (GENIE_OBJ_FORM, 0, 0);

	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_1, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_2, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_3, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_4, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_5, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_6, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_7, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_8, 0);

	// init

	for(i = 0; i < channels; i++)
	{
		alarm_activated[i] = 0;
		genieWriteObj(GENIE_OBJ_USER_LED, i, 0);
		genieWriteObj(GENIE_OBJ_4DBUTTON, rocker[i], 0);
	}

	fp = fopen(data_file, "r+");

	line = calloc(line_length, sizeof(char));


	fgets(line, line_length, fp);
	fgets(line, line_length, fp);
	// printf("%s\n", line);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		gradient[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		offset[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		max[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		min[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		ref_volt_1[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		ref_volt_2[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		alarm_max[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		alarm_min[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}

	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

	token = strtok(line, t);
	i = 0;
	while (token)
	{
		armed[i] = atof(token);
		token = strtok(NULL, t);
		if (i < channels - 1)
		{
			i++;
		}
		else
		{
			break;
		}
	}


	line = malloc(line_length * sizeof(char));

	fgets(line, line_length, fp);
	fgets(line, line_length, fp);

    volume = atoi(line);
    printf("volume: %d\n", volume);

    // volume
	genieWriteObj(GENIE_OBJ_SOUND, 1, volume);

	fclose(fp);
	return 0;
}

/*
 * adc_read_loop:
 *  Read adc values within a separate thread.
 *********************************************************************************
 */

static void *adc_read_loop (void *data)
{
	int j;
	struct sched_param sched;
	int pri = 10;
	int temp_form;

	// Set to a real-time priority
	//  (only works if root, ignored otherwise)

	memset (&sched, 0, sizeof(sched));

	if (pri > sched_get_priority_max (SCHED_RR))
		pri = sched_get_priority_max (SCHED_RR);

	sched.sched_priority = pri;
	sched_setscheduler (0, SCHED_RR, &sched);

	// sleep(1);
	for (;;)
	{
		for (j = 0; j < 8; j++)
		{
			// here we obtain the true voltage from the adc and convert it to
			true_voltage[j] = getadc(j + 1);
			modified_voltage[j] = gradient[j] * true_voltage[j] + offset[j];
			// printf ("Channel: %d  = %2.4fV\n", j + 1, modified_voltage[j]);

			if (alarm_max[j] > alarm_min[j])
			{
				if (modified_voltage[j] > alarm_max[j] || modified_voltage[j] < alarm_min[j])
				{
					if (armed[j])
					{
						alarm_activated[j] = 1;
						// genieWriteObj(GENIE_OBJ_SOUND, 0, j + 2);
						genieWriteObj(GENIE_OBJ_SOUND, 0, 8 - j);
						if (current_form != ALARM)
						{
							genieWriteObj(GENIE_OBJ_FORM, ALARM, 0);
							updateForm(ALARM);
						}
					}
				}
				else
				{
					if (alarm_activated[j])
					{
						alarm_activated[j] = 0;
						if (current_form == ALARM)
						{
							genieWriteObj(GENIE_OBJ_FORM, previous_form, 0);
							temp_form = current_form;
							current_form = previous_form;
							previous_form = temp_form;
						}
					}
				}
			}
			else
			{
				if (modified_voltage[j] < alarm_max[j] || modified_voltage[j] > alarm_min[j])
				{
					if (armed[j])
					{

						genieWriteObj(GENIE_OBJ_SOUND, 0, 8 - j);
						alarm_activated[j] = 1;
						if (current_form != ALARM)
						{
							genieWriteObj(GENIE_OBJ_FORM, ALARM, 0);
							updateForm(ALARM);
						}
					} 
				} 
				else
				{
					if (alarm_activated[j])
					{
						alarm_activated[j] = 0;
						if (current_form == ALARM)
						{
							genieWriteObj(GENIE_OBJ_FORM, previous_form, 0);
							temp_form = current_form;
							current_form = previous_form;
							previous_form = temp_form;
						}
					}
				}
			}

			updateDisplay(modified_voltage[j], j);
			
			// genieWriteObj(GENIE_OBJ_SCOPE, j < 4 ? 0 : 1, (int)(true_voltage[j]*25 + 50));
		}
		// printf("\n");
	}

	return (void *)NULL;
}

/*
 * handleGenieEvent:
 *  Take a reply off the display and call the appropriate handler for it.
 *********************************************************************************
 */

void handleGenieEvent (struct genieReplyStruct *reply)
{
	int i = 0;
	int slider_exists = FALSE;
	int rocker_exists = FALSE;
	int temp_form;

	static int save_volume = 0;

	if (reply->cmd != GENIE_REPORT_EVENT)
	{
		printf ("Invalid event from the display: 0x%02X\r\n", reply->cmd);
		return;
	}
	
	if (reply->object == GENIE_OBJ_TRACKBAR && reply->index == 0)
	{
		volume = reply->data;
		save_volume = 1;
		return;
	}

    // workaround so that one doesn't save volume to flash every time that one changes it..
	if (save_volume)
	{
		save_volume = 0;
		printf("volume: %d\n", volume);
		save_to_file();
	}

	if (reply->object == GENIE_OBJ_FORM)
	{
		updateForm(reply->index);
		if (reply->index == SETUP_ALARM)
		{
			updateAlarm();
		}
	}

	switch (current_form)
	{
	case HOME:
		puts("HOME");
		if (previous_form == NUMPAD)
		{
			processKey('c');
		}
		break;

	case CALIBRATE:
		puts("CALIBRATE");

		if (reply->object == GENIE_OBJ_WINBUTTON)
		{
			switch (reply->index)
			{
			case BUT_GRAD:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_GRAD;
				break;
			case BUT_OFFS:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_OFFS;
				break;
				/*  				case BUT_AUTO:
					genieWriteObj (GENIE_OBJ_FORM, AUTO, 0);
					updateForm(AUTO);
				break;
				case BUT_RESET:
					genieWriteObj (GENIE_OBJ_FORM, CONFIRMATION, 0);
					updateForm(CONFIRMATION);
				break;*/
			case BUT_MAX:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_MAX;
				break;
			case BUT_MIN:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_MIN;
				break;
			}
		}
		else if (reply->object == GENIE_OBJ_4DBUTTON)
		{
			switch(reply->index)
			{
			case BUT_4D_RESET:
				genieWriteObj (GENIE_OBJ_FORM, CONFIRMATION, 0);
				updateForm(CONFIRMATION);
				break;
			}

			for (i = 0; i < channels; i++)
			{
				if (reply->index == slider[i])
				{
					slider_exists = TRUE;
					current_slider = i;
					break;
				}
			}

			if (slider_exists)
			{
				if (reply->data == 1)
				{
					slider_values[current_slider] = 1;
				}
				else
				{
					slider_values[current_slider] = 0;	
					current_slider = -1;
				}
			}
		}

		/*for (i = 0; i < channels; i++)
		{
			printf("%d: %d\n", i, slider_values[i]);
			if (slider_values[i])
			{

			}
		}
		printf("%d\n", current_slider);*/

		updateGraphFormula();
		updateRange();

		break;

	case NUMPAD:
		puts("NUMPAD");
		// printf("previous_form: %d\n", previous_form);
		if (reply->object == GENIE_OBJ_WINBUTTON)
		{
			if (reply->index == BUT_KB_BACK)
			{
				processKey('c');
				if (previous_form == CALIBRATE)
				{
					genieWriteObj(GENIE_OBJ_FORM, CALIBRATE, 0);
					updateForm(CALIBRATE);
					updateGraphFormula();
					updateRange();
				}
				else if (previous_form == AUTO)
				{
					genieWriteObj(GENIE_OBJ_FORM, AUTO, 0);
					updateForm(AUTO);	
					updateAutoScreen();
				}
				else if (previous_form == SETUP_ALARM)
				{
					genieWriteObj(GENIE_OBJ_FORM, SETUP_ALARM, 0);
					updateForm(SETUP_ALARM);	
					updateAlarm();
				}
				else if (previous_form == ALARM)
				{
					genieWriteObj(GENIE_OBJ_FORM, pre_previous_form, 0);
					updateForm(pre_previous_form);	
					updateAlarm();
					updateAutoScreen();
					updateGraphFormula();
					updateRange();
				}
			}
		}
		else if (reply->object == GENIE_OBJ_KEYBOARD)
		{
			if (reply->index == 0)  // Only one keyboard
				processKey(reply->data);
			else
				printf ("Unknown keyboard: %d\n", reply->index);
		}
		break;

	case AUTO:
		puts("AUTO");

		updateAutoScreen();

		if (reply->object == GENIE_OBJ_WINBUTTON)
		{
			switch (reply->index)
			{
			case BUT_CH_1:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_CH_1;
				break;
			case BUT_CH_2:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_CH_2;
				break;
			case BUT_SAVE_1:
				puts("SAVE_1");
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						true_volt_1[i] = true_voltage[i];
						gradient[i] = (ref_volt_1[i] - ref_volt_2[i]) / (true_volt_1[i] - true_volt_2[i]);
						offset[i] = ref_volt_1[i] - gradient[i] * true_volt_1[i];
						max[i] = gradient[i] * max_volt + offset[i];
						min[i] = gradient[i] * min_volt + offset[i];
					}
				}
				save_to_file();

				break;
			case BUT_SAVE_2:
				puts("SAVE_2");
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						true_volt_2[i] = true_voltage[i];
						gradient[i] = (ref_volt_2[i] - ref_volt_1[i]) / (true_volt_2[i] - true_volt_1[i]);
						offset[i] = ref_volt_2[i] - gradient[i] * true_volt_2[i];
						max[i] = gradient[i] * max_volt + offset[i];
						min[i] = gradient[i] * min_volt + offset[i];
					}
				}
				save_to_file();
				break;
			}
		}
		break;

	case CONFIRMATION:
		puts("CONFIRMATION");
		if (reply->object == GENIE_OBJ_WINBUTTON)
		{
			if (reply->index == BUT_YES)
			{
				reset();
				genieWriteObj(GENIE_OBJ_FORM, CALIBRATE, 0);
				updateForm(CALIBRATE);
				updateNumpadDisplay();
			}
			else if (reply->index == BUT_NO)
			{
				genieWriteObj(GENIE_OBJ_FORM, CALIBRATE, 0);
				updateForm(CALIBRATE);
				updateNumpadDisplay();
			}
		}
		break;

	case SETUP_ALARM:
		puts("SETUP_ALARM");

		if (reply->object == GENIE_OBJ_WINBUTTON)
		{
			switch(reply->index)
			{
			case BUT_ALARM_ARM:
				for (i = 0; i < channels; i++)
				{
					if (rocker_values[i])
					{
						armed[i] = 1;
						genieWriteObj(GENIE_OBJ_USER_LED, i, 1);
					}
				}
				break;

			case BUT_ALARM_DISARM:
				for (i = 0; i < channels; i++)
				{
					if (rocker_values[i])
					{
						armed[i] = 0;
						genieWriteObj(GENIE_OBJ_USER_LED, i, 0);
					}
				}
				break;

			case BUT_ALARM_MIN:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_ALARM_MIN;
				break;

			case BUT_ALARM_MAX:
				genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0);
				updateForm(NUMPAD);
				last_edit_button = BUT_ALARM_MAX;
				break;

			case BUT_ALARM_RESET:
				reset_alarm_min_max();
				break;
			}
		}
		else if (reply->object == GENIE_OBJ_4DBUTTON)
		{
			for (i = 0; i < channels; i++)
			{
				if (reply->index == rocker[i])
				{
					// rocker_exists = TRUE;
					rocker_values[i] = reply->data;
					break;
				}
			}

			/*if (rocker_exists)
		{
		  if (reply->data == 1)
		  {
			rocker_values[reply->index] = 1;
		  }
		  else
		  {
			rocker_values[reply->index] = 0;
		  }
		}*/
		}
		break;

		case ALARM:
			if (reply->object == GENIE_OBJ_WINBUTTON)
			{
				if (reply->index == BUT__ALARM)
				{
					for (i = 0; i < channels; i++)
					{
						if (alarm_activated[i])
						{
							armed[i] = 0;
							alarm_activated[i] = 0;
							genieWriteObj(GENIE_OBJ_USER_LED, i, 0);
						}
					}
				}
				else if (reply->index == BUT__ALARM_DISARM_ALL)
				{
					for (i = 0; i < channels; i++)
					{
						armed[i] = 0;
						alarm_activated[i] = 0;
						genieWriteObj(GENIE_OBJ_USER_LED, i, 0);
					}
				}
				genieWriteObj(GENIE_OBJ_FORM, previous_form, 0);
				// updateForm(previous_form);
				temp_form = current_form;
				current_form = previous_form;
				previous_form = temp_form;
				// printf("%d, %d, %d\n", pre_previous_form, previous_form, current_form);
				
				updateGraphFormula();
				updateRange();
				updateAlarm();
				updateNumpadDisplay();
				updateAutoScreen();
			}
		break;

		case SETTINGS:
		    if (reply->object == GENIE_OBJ_WINBUTTON)
		    {
		    	if (reply->index == BUT_REBOOT)
		    	{
		    	    puts("System going down for reboot now!");
		    	    system("sudo reboot");
		    	}
		    	if (reply->index == BUT_SHUTDOWN)
		    	{
		    	    puts("System going down for shutdown now!");
		    	    system("sudo halt");
		    	}
		    }
	    break;
	}

	/*
	else
	  printf ("Unknown button: %d\n", reply->index);
  }
  else
	printf ("Unhandled Event: object: %2d, index: %d data: %d [%02X %02X %04X]\r\n",
			reply->object, reply->index, reply->data, reply->object, reply->index, reply->data);
	 */
}

void updateForm(int form)
{
	pre_previous_form = previous_form;
	previous_form = current_form;
	current_form = form;
	// printf("%d, %d, %d\n", pre_previous_form, previous_form, current_form);
}

/*
 *  processKey:
 *  A key has been pressed on the keyboard.
 *********************************************************************************
 */

void processKey (int key)
{
	static int minus = FALSE;
	int i;

	double numberDouble;

	char hyphen[display_length];

	if (isdigit (key))
	{
		sprintf(numberString, "%s%c", numberString, key);
		// printf("numberString: %s\n", numberString);

		updateNumpadDisplay ();
		return;
	}

	switch (key)
	{

	case 'c':     // Clear entry
		memset(numberString, 0, display_length);
		minus = FALSE;
		break;

		// Other functions

	case KB_SIGN_CHANGE:   // +/-
		if (!minus)
		{
			hyphen[0] = '-';
			hyphen[1] = '\0';
			strcat(hyphen, numberString);
			strcpy(numberString, hyphen);
			minus = TRUE;
		}
		else
		{
			strcpy(numberString, numberString + 1);
			minus = FALSE;
		}
		break;

		// Operators

	case KB_DOT:
		sprintf(numberString, "%s%c", numberString, '.');
		break;

	case KB_BACKSPACE:

		numberString[strlen(numberString)-1] = '\0';
		if (strlen(numberString) < 1)
		{
			minus = FALSE;
		}
		// updateNumpadDisplay ();
		break;

	case KB_SAVE:
		if (isNumeric(numberString))
		{
			numberDouble = atof(numberString);
			sprintf(numberString, "%lf", numberDouble);
			switch (last_edit_button)
			{
			case BUT_GRAD:
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						gradient[i] = numberDouble;
						max[i] = gradient[i] * max_volt + offset[i];
						min[i] = gradient[i] * min_volt + offset[i];
					}
				}
				break;
			case BUT_OFFS:
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						offset[i] = numberDouble;
						max[i] = gradient[i] * max_volt + offset[i];
						min[i] = gradient[i] * min_volt + offset[i];
					}
				}
				break;
			case BUT_MAX:
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						max[i] = numberDouble;
					}
				}
				break;
			case BUT_MIN:
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						min[i] = numberDouble;
					}
				}
				break;
			case BUT_CH_1:
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						ref_volt_1[i] = numberDouble;
					}
				}
				break;
			case BUT_CH_2:
				for (i = 0; i < channels; i++)
				{
					if (slider_values[i])
					{
						ref_volt_2[i] = numberDouble;
					}
				}
				break;
			case BUT_ALARM_MIN:
				for (i = 0; i < channels; i++)
				{
					if (rocker_values[i])
					{
						alarm_min[i] = numberDouble;
					}
				}
				break;
			case BUT_ALARM_MAX:
				for (i = 0; i < channels; i++)
				{
					if (rocker_values[i])
					{
						alarm_max[i] = numberDouble;
					}
				}
				break;
			}
			save_to_file();
		}
		else
		{
			// todo: make so that any key removes error
			sprintf(numberString, "ERROR");
		}
		break;

	default:
		printf ("*** Unknown key from display: 0x%02X, %d\n", key, key);
		break;
	}

	updateNumpadDisplay ();
	// printf("numberString: %s\n", numberString);
}


/*
 * updateDisplay:
 *  Do just that.
 *********************************************************************************
 */

void updateDisplay (double val, int index)
{
	char buf[32];

	double output;
	double graph_gradient;
	double graph_offset;

	graph_gradient = 100 / (max[index] - min[index]);
	graph_offset = 100 - graph_gradient * max[index];
	output = graph_gradient * val + graph_offset;

	sprintf(buf, "%.10lf V", val);
	genieWriteStr(index, buf);

	genieWriteObj(GENIE_OBJ_SCOPE, index < 4 ? 0 : 1, (int)(output));
	// if (index == 0)
	// { 
	//   printf("%d: %lf  grad: %lf, offs: %lf\n", index, output, graph_gradient, graph_offset);
	// }

	// look at this:
	// initial attempt at graph equation
	// ((val - offset[index]) / (temp_diff / 4))*25 + 50 - (true_avg * 50)
	// drawing on paper first makes life much easier

}

/*
 * updateNumpadDisplay:
 *  Do just that.
 *********************************************************************************
 */

void updateNumpadDisplay (void)
{
	char buf[32];

	if (errorCondition)
		sprintf (buf, "%s", "ERROR");
	else
	{
		// sprintf (buf, "%13.13g", display);
		strcpy(buf, numberString);
		// printf ("%s\n", buf);
	}

	genieWriteStr (17, buf);  // Text box number 17
}

/*
 * updateGraphFormula:
 *  Do just that.
 *********************************************************************************
 */

void updateGraphFormula (void)
{
	char buf[32];

	if (errorCondition)
		sprintf (buf, "%s", "ERROR");
	else
	{
		if (current_slider == -1)
		{
			strcpy(buf, "        y = m * x + c");
		}
		else
		{
			sprintf (buf, "y = %6.4lfx + %5.3lf", gradient[current_slider], offset[current_slider]);
			// printf ("%s\n", buf);
		}
	}

	genieWriteStr (16, buf);  // Text box number 16
}

/*
 * updateRange:
 *  Do just that.
 *********************************************************************************
 */

void updateRange (void)
{
	char buf[40];

	if (errorCondition)
		sprintf (buf, "%s", "ERROR");
	else
	{
		if (current_slider == -1)
		{
			strcpy(buf, "Scope Range:\n[-2, 2] V");
		}
		else
		{
			sprintf (buf, "[%lf,\n %lf] V", min[current_slider], max[current_slider]);
			// printf ("%s\n", buf);
		}
	}

	genieWriteStr (21, buf);  // Text box number 16
}


/*
 * updateAutoScreen:
 *  Do just that.
 *********************************************************************************
 */

void updateAutoScreen (void)
{
	char buf_1[32];
	char buf_2[32];

	if (errorCondition)
	{
		sprintf (buf_1, "%s", "ERROR");
		sprintf (buf_2, "%s", "ERROR");
	}
	else
	{
		if (current_slider == -1)
		{
			strcpy(buf_1, "             0 V");
			strcpy(buf_2, "             12 V");
		}
		else
		{
			sprintf (buf_1, "%lf V", ref_volt_1[current_slider]);
			sprintf (buf_2, "%lf V", ref_volt_2[current_slider]);
			// printf ("%s\n", buf);
		}
	}

	genieWriteStr (19, buf_1);  // Text box number 19
	genieWriteStr (20, buf_2);  // Text box number 20
}

/*
 * updateAlarm:
 *  Update min/max values on alarm screen.
 *********************************************************************************
 */

void updateAlarm (void)
{
	char buf[32];
	int i;

	if (errorCondition)
	{
		sprintf (buf, "%s", "ERROR");
	}
	else
	{
		for (i = 0; i < channels; i++)
		{
			sprintf (buf, "%lf V", alarm_min[i]);
			genieWriteStr (i + 33, buf);

			sprintf (buf, "%lf V", alarm_max[i]);
			genieWriteStr (i + 42, buf);
		}

	}

}

/*
 * isNumeric:
 *  Check if a number is numeric.
 *
 *  @return: 1 if true or 0 if false.
 *********************************************************************************
 */

int isNumeric(char *str)
{
	int dot = 0;
	int minus = 0;

	while(*str)
	{
		if (*str == '\0')
		{
			return 0;
		}
		else if (*str == '.')
		{
			dot++;
			if (dot > 1)
			{
				return 0;
			}
		}
		else if (*str == '-')
		{
			minus++;
			if (minus > 1)
			{
				return 0;
			}
		}
		else if (!isdigit(*str))
		{
			return 0;
		}
		str++;
	}
	return 1;
}

void reset(void)
{
	int i;

	for (i = 0; i < 8; i++)
	{
		gradient[i] = 1;
		offset[i] = 0;
		max[i] = max_volt;
		min[i] = min_volt;
		ref_volt_1[i] = 0;
		ref_volt_2[i] = 12;
	}

	save_to_file();
}

void reset_alarm_min_max(void)
{
	int i;

	for (i = 0; i < 8; i++)
	{
		alarm_max[i] = 5;
		alarm_min[i] = -5;
	}

	save_to_file();
}

void save_to_file(void)
{
	int i;
	char *line;
	fp = fopen(data_file, "w");

	fprintf(fp, "gradient:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", gradient[i]);
	}

	fprintf(fp, "\noffset:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", offset[i]);
	}

	fprintf(fp, "\nmax:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", max[i]);
	}

	fprintf(fp, "\nmin:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", min[i]);
	}

	fprintf(fp, "\nref_volt_1:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", ref_volt_1[i]);
	}

	fprintf(fp, "\nref_volt_2:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", ref_volt_2[i]);
	}

	fprintf(fp, "\nalarm_max:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", alarm_max[i]);
	}

	fprintf(fp, "\nalarm_min:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", alarm_min[i]);
	}

	fprintf(fp, "\narmed:\n");

	for (i = 0; i < channels; i++)
	{
		fprintf(fp, "%lf,", armed[i]);
	}

	fprintf(fp, "\nvolume:\n");
    fprintf(fp, "%d", volume);

	fclose(fp);
}