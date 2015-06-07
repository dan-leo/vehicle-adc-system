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

#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include <wiringPi.h>
#include <softPwm.h>

#include <geniePi.h>

#include "adcpiv3.h"

void updateDisplay (float val, int index);
int setup(void);

int main(int argc, char **argv) {
	int i, j;
	float true_voltage[8];
	int channel;
	setup();
	genieWriteStr (0, "hi");
	genieWriteStr (1, "there");
	// setup multiplier based on input voltage range and divisor true_voltageue
	varMultiplier = (1 / varDivisior) / 1000;
	// 2.4705882 constant in place of 1 (not neccessary)

	if (argc > 1) channel = atoi(argv[1]);
	if (channel < 1 | channel > 8) channel = 1;
	// loop for 500 samples and print to terminal
	// for (i = 0; i < 500; i++) {
	//   true_voltage = getadc (channel);
	//   if (true_voltage <= 5.5) {
	//     printf ("Channel: %d  - %2.4fV\n", channel, true_voltage);
	//   }
	//   sleep (0.5);
	// }
	for (i = 0; i < 500; i++) {
		for (j = 0; j < 8; j++)
		{
			true_voltage[j] = getadc(j + 1);
			if (true_voltage[j] <= 5.5) {
				printf ("Channel: %d  = %2.4fV\n", j + 1, true_voltage[j]);
				updateDisplay(true_voltage[j], j);
			}
			sleep (0.001);
		}
		puts("");
		sleep(0.5);
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
	if (index < 4) genieWriteObj(GENIE_OBJ_SCOPE, 0, (int)(val*50));
}

int setup(void)
{
	// Genie display setup
	// Using the Raspberry Pi's on-board serial port.
	if (genieSetup ("/dev/ttyAMA0", 115200) < 0)
	{
		fprintf (stderr, "rgb: Can't initialise Genie Display: %s\n", strerror (errno)) ;
		return 1 ;
	}

	// Select form 0, Home
	genieWriteObj (GENIE_OBJ_FORM, 0, 0) ;
	return 0;
}