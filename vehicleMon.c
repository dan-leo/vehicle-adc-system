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

float true_voltage[8];
float modified_voltage[8];
float gradient[8];
float offset[8];
int current_form, previous_form;

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

enum win_button
{
	BUT_GRAD = 2,
	BUT_OFFS = 3,
	BUT_KB_BACK = 8,
	BUT_MAX = 18,
	BUT_MIN = 17
};

enum button_4D
{
	BUT_4D_AUTO = 11,
	BUT_4D_RESET = 12
};


void updateDisplay (float val, int index);
int setup(void);
static void *adc_read_loop (void *data);
void handleGenieEvent (struct genieReplyStruct *reply);
void updateForm(int form);

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

/*
 * updateDisplay:
 *  Do just that.
 *********************************************************************************
 */
 
void updateDisplay (float val, int index)
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

	sprintf(buf, "%f", val);
	genieWriteStr(index, buf);

	genieWriteObj(GENIE_OBJ_SCOPE, index < 4 ? 0 : 1, (int)(val*25 + 50));

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

	// init
	for (i = 0; i < 8; i++)
	{
		gradient[i] = 1;
		offset[i] = 0;
	}
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

			printf ("Channel: %d  = %2.4fV\n", j + 1, modified_voltage[j]);
			updateDisplay(modified_voltage[j], j);
		}
		printf("\n");
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
  				break;
  				case BUT_OFFS:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
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
  				break;
  				case BUT_MIN:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
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
  		}
  	break;

  	case NUMPAD:
  		puts("NUMPAD");
  		printf("previous_form: %d\n", previous_form);
  		if (reply->object == GENIE_OBJ_WINBUTTON)
  		{
  			if (reply->index == BUT_KB_BACK)
  			{
  				genieWriteObj(GENIE_OBJ_FORM, CALIBRATE, 0);
  				updateForm(CALIBRATE);
  			}
  		}
  	break;

  	case AUTO:
  		puts("AUTO");
  		if (reply->object == GENIE_OBJ_WINBUTTON)
  		{
  			switch (reply->index)
  			{
  				case BUT_GRAD:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
  				break;
  				case BUT_OFFS:
  					genieWriteObj (GENIE_OBJ_FORM, NUMPAD, 0) ;
  					updateForm(NUMPAD);
  				break;
  			}
  		}
  	break;

  	case CONFIRMATION:
  		puts("CONFIRMATION");
  	break;
  }
  
  /*
  if (reply->object == GENIE_OBJ_KEYBOARD)
  {
    if (reply->index == 0)  // Only one keyboard
      // calculatorKey (reply->data) ;
    	printf("");
    else
      printf ("Unknown keyboard: %d\n", reply->index) ;
  }
  else if (reply->object == GENIE_OBJ_WINBUTTON)
  {
    if (reply->index == 1) // Clock button on main display
      genieWriteObj (GENIE_OBJ_FORM, 1, 0) ;
    else if (reply->index == 0) // Calculator button on clock display
    {
      genieWriteObj (GENIE_OBJ_FORM, 0, 0) ;
      // updateDisplay () ;
    }
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