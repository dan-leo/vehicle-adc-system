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


int current_form, previous_form;
int errorCondition;
int slider_values[channels];
int current_slider = -1;
int last_edit_button;

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

char display[display_length];
char numberString[display_length];

enum op_form 
{
	HOME,
	SCOPE,
	CALIBRATE,
	NUMPAD,
	CONFIRMATION,
	AUTO
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
	CH_8 = 0
};

int slider[channels] = {CH_1, CH_2, CH_3, CH_4 , CH_5 , CH_6 , CH_7 , CH_8};

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
	BUT_MIN = 17
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
void processKey (int key);
int isNumeric(char *str);
void reset(void);

/*
 *********************************************************************************
 * main:
 *  Run the touchscreen adc
 *********************************************************************************
 */

int main(int argc, char **argv) {
	int i, j, channel;
	pthread_t myThread;
	struct genieReplyStruct reply ;


	setup();

	// setup multiplier based on input voltage range and divisor
	// removed 2.4705882 constant in place of 1 
	varMultiplier = (1 / varDivisior) / 1000;
	
	// possibly useful code
	if (argc > 1) channel = atoi(argv[1]);
	if (channel < 1 | channel > 8) channel = 1;

	// start adc read thread
	(void)pthread_create (&myThread, NULL, adc_read_loop, NULL);

	// loop and print to terminal
	for (;;)
	{
		while (genieReplyAvail ())
    	{
	    	genieGetReply    (&reply) ;
      		handleGenieEvent (&reply) ;
    	}
    	usleep (10000) ; // 10mS - Don't hog the CPU in-case anything else is happening...
	}
	return 0;
}

int setup(void)
{
	int i;
	// Genie display setup
	// Using the Raspberry Pi's on-board serial port.
	if (genieSetup ("/dev/ttyAMA0", 115200) < 0)
	{
		fprintf (stderr, "rgb: Can't initialise Genie Display: %s\n", strerror (errno)) ;
		return 1 ;
	}

	// Select form 0, Home
	genieWriteObj (GENIE_OBJ_FORM, 0, 0) ;

	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_1, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_2, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_3, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_4, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_5, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_6, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_7, 0);
	genieWriteObj (GENIE_OBJ_4DBUTTON, CH_8, 0);

	/*memset(ref_volt_1, 0, channels);
	memset(ref_volt_2, 12, channels);*/

	// init
	reset();
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
  struct sched_param sched ;
  int pri = 10 ;

  // Set to a real-time priority
  //  (only works if root, ignored otherwise)

  memset (&sched, 0, sizeof(sched)) ;

  if (pri > sched_get_priority_max (SCHED_RR))
    pri = sched_get_priority_max (SCHED_RR) ;

  sched.sched_priority = pri ;
  sched_setscheduler (0, SCHED_RR, &sched) ;

  // sleep(1);

  for (;;)
  {
  	for (j = 0; j < 8; j++)
		{
			true_voltage[j] = getadc(j + 1);
			modified_voltage[j] = gradient[j] * true_voltage[j] + offset[j];

			// printf ("Channel: %d  = %2.4fV\n", j + 1, modified_voltage[j]);
			updateDisplay(modified_voltage[j], j);
		}
		// printf("\n");
  }

  return (void *)NULL ;
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

  if (reply->cmd != GENIE_REPORT_EVENT)
  {
    printf ("Invalid event from the display: 0x%02X\r\n", reply->cmd) ;
    return ;
  }

  if (reply->object == GENIE_OBJ_FORM)
  {
  	updateForm(reply->index);
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
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
  					last_edit_button = BUT_GRAD;
  				break;
  				case BUT_OFFS:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
  					last_edit_button = BUT_OFFS;
  				break;
/*  				case BUT_AUTO:
  					genieWriteObj (GENIE_OBJ_FORM, AUTO, 0) ;
  					updateForm(AUTO);
  				break;
  				case BUT_RESET:
  					genieWriteObj (GENIE_OBJ_FORM, CONFIRMATION, 0) ;
  					updateForm(CONFIRMATION);
  				break;*/
  				case BUT_MAX:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
  					last_edit_button = BUT_MAX;
  				break;
  				case BUT_MIN:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
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
  					genieWriteObj (GENIE_OBJ_FORM, CONFIRMATION, 0) ;
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
  				}
  				else if (previous_form == AUTO)
  				{
  					genieWriteObj(GENIE_OBJ_FORM, AUTO, 0);
  					updateForm(AUTO);	
  					updateAutoScreen();
  				}
  			}
  		}
  		else if (reply->object == GENIE_OBJ_KEYBOARD)
  		{
  			if (reply->index == 0)  // Only one keyboard
      			processKey(reply->data) ;
    		else
      			printf ("Unknown keyboard: %d\n", reply->index) ;
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
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
  					last_edit_button = BUT_CH_1;
  				break;
  				case BUT_CH_2:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
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
	  					}
  					}

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
	  					}
  					}
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
  }
  
  /*
    else
      printf ("Unknown button: %d\n", reply->index) ;
  }
  else
    printf ("Unhandled Event: object: %2d, index: %d data: %d [%02X %02X %04X]\r\n",
            reply->object, reply->index, reply->data, reply->object, reply->index, reply->data);
  */
}

void updateForm(int form)
{
  	previous_form = current_form;
 	current_form = form;
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
    printf("numberString: %s\n", numberString);
    
    updateNumpadDisplay () ;
    return ;
  }

  switch (key)
  {

  case 'c':     // Clear entry
  	  memset(numberString, 0, display_length);
  	  minus = FALSE;
    break ;

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
    break ;

  // Operators

  case KB_DOT:
  	sprintf(numberString, "%s%c", numberString, '.');
    break ;

  case KB_BACKSPACE:
    
  	numberString[strlen(numberString)-1] = '\0';
  	if (strlen(numberString) < 1)
  	{
  		minus = FALSE;
  	}
    // updateNumpadDisplay () ;
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
    				}
    			}
    		break;
    		case BUT_OFFS:
    			for (i = 0; i < channels; i++)
    			{
    				if (slider_values[i])
    				{
    					offset[i] = numberDouble;
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
    	}

    }
    else
    {
    	// todo: make so that any key removes error
    	sprintf(numberString, "ERROR");
    }
  	break;

  default:
    printf ("*** Unknown key from display: 0x%02X, %d\n", key, key) ;
    break ;
  }

  updateNumpadDisplay () ;
  printf("numberString: %s\n", numberString);
}


/*
 * updateDisplay:
 *  Do just that.
 *********************************************************************************
 */
 
void updateDisplay (double val, int index)
{
	char buf [32];
	/*if (errorCondition)
	  sprintf (buf, "%s", "ERROR") ;
	else
	{
	  sprintf (buf, "%11.9g", display) ;
	  printf ("%s\n", buf) ;
	}*/

	/*sprintf (buf, "%c %c",
	         memory != 0.0 ? 'M' : ' ',
	         lastOperator == 0 ? ' ' : lastOperator) ;

	genieWriteStr (1, buf) ;  // Text box number 1*/

	sprintf(buf, "%.10lf", val);
	genieWriteStr(index, buf);

	genieWriteObj(GENIE_OBJ_SCOPE, index < 4 ? 0 : 1, (int)(val*25 + 50));

}

/*
 * updateNumpadDisplay:
 *  Do just that.
 *********************************************************************************
 */

void updateNumpadDisplay (void)
{
  char buf [32] ;

  if (errorCondition)
    sprintf (buf, "%s", "ERROR") ;
  else
  {
    // sprintf (buf, "%13.13g", display) ;
    strcpy(buf, numberString);
    // printf ("%s\n", buf) ;
  }

  genieWriteStr (17, buf) ;  // Text box number 17
}

/*
 * updateGraphFormula:
 *  Do just that.
 *********************************************************************************
 */

void updateGraphFormula (void)
{
  char buf [32] ;

  if (errorCondition)
    sprintf (buf, "%s", "ERROR") ;
  else
  {
  	if (current_slider == -1)
  	{
		strcpy(buf, "        y = m * x + c");
  	}
  	else
  	{
    	sprintf (buf, "y = %6.4lfx + %5.3lf", gradient[current_slider], offset[current_slider]);
    	// printf ("%s\n", buf) ;
    }
  }

  genieWriteStr (16, buf) ;  // Text box number 16
}


/*
 * updateAutoScreen:
 *  Do just that.
 *********************************************************************************
 */

void updateAutoScreen (void)
{
  char buf_1 [32] ;
  char buf_2 [32] ;

  if (errorCondition)
  {
    sprintf (buf_1, "%s", "ERROR") ;
	sprintf (buf_2, "%s", "ERROR") ;
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
    	// printf ("%s\n", buf) ;
    }
  }

  genieWriteStr (19, buf_1) ;  // Text box number 19
  genieWriteStr (20, buf_2) ;  // Text box number 20
}

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
		max[i] = 2;
		min[i] = -2;
		ref_volt_1[i] = 0;
		ref_volt_2[i] = 12;
	}
}