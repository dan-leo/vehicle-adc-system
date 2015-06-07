/*
read abelectronics ADC Pi V2 board inputs with repeating reading from each channel.

Based on ADC 1 demo code by Stephan Callsen from https://github.com/abelectronicsuk/adcpi/blob/master/adcon.c

Author: Brian Dorey
Version:  1.0
Date: 15 August 2013
Version History:
1.0 - Initial Release

  Required package:
  apt-get install libi2c-dev

  Compile with gcc:
  gcc adc.c -o adc

  Execute with:
  ./adc [channel]
  default is channel 1

*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#include "adcpiv3.h"

const float varDivisior = 64;
float varMultiplier = 0;

int main1(int argc, char **argv) {
  int i, j;
  float val;
  int channel;
  // setup multiplier based on input voltage range and divisor value
  varMultiplier = (1 / varDivisior) / 1000;
  // 2.4705882 constant in place of 1 (not neccessary)

  if (argc > 1) channel = atoi(argv[1]);
  if (channel < 1 | channel > 8) channel = 1;
  // loop for 500 samples and print to terminal
  /*for (i = 0; i < 500; i++) {
    val = getadc (channel);
    if (val <= 5.5) {
      printf ("Channel: %d  - %2.4fV\n", channel, val);
    }
    sleep (0.5);
  }*/
  for (i = 0; i < 500; i++) {
    for (j = 1; j <= 8; j++)
    {
      val = getadc (j);
      printf ("Channel: %d  - %2.4fV\n", j, val);
      /*if (val <= 5.5) {
        printf ("Channel: %d  - %2.4fV\n", j, val);
      }*/
      sleep (0.001);
    }
    puts("");
    sleep(0.5);
  }
  return 0;
}

float getadc (int chn) {
  unsigned int fh, dummy, adc, adc_channel;
  float val;
  __u8  res[4];
  // select chip and channel from args
  switch (chn) {
  case 1: { adc = ADC_1; adc_channel = ADC_CHANNEL1; }; break;
  case 2: { adc = ADC_1; adc_channel = ADC_CHANNEL2; }; break;
  case 3: { adc = ADC_1; adc_channel = ADC_CHANNEL3; }; break;
  case 4: { adc = ADC_1; adc_channel = ADC_CHANNEL4; }; break;
  case 5: { adc = ADC_2; adc_channel = ADC_CHANNEL1; }; break;
  case 6: { adc = ADC_2; adc_channel = ADC_CHANNEL2; }; break;
  case 7: { adc = ADC_2; adc_channel = ADC_CHANNEL3; }; break;
  case 8: { adc = ADC_2; adc_channel = ADC_CHANNEL4; }; break;
  default: { adc = ADC_1; adc_channel = ADC_CHANNEL1; }; break;
  }
  // open /dev/i2c-0 for version 1 Raspberry Pi boards
  // open /dev/i2c-1 for version 2 Raspberry Pi boards
  fh = open("/dev/i2c-1", O_RDWR);
  ioctl(fh, I2C_SLAVE, adc);
  // send request for channel
  i2c_smbus_write_byte (fh, adc_channel);
  usleep (50000);
  // read 4 bytes of data
  i2c_smbus_read_i2c_block_data(fh, adc_channel, 4, res);
  // loop to check new value is available and then return value
  while (res[3] & 128) {
    // read 4 bytes of data
    i2c_smbus_read_i2c_block_data(fh, adc_channel, 4, res);
  }
  usleep(50000);
  close (fh);

  // shift bits to product result
  dummy = ((res[0] & 0b00000001) << 16) | (res[1] << 8) | res[2];

  // check if positive or negative number and invert if needed
  if (res[0] >= 128) dummy = ~(0x020000 - dummy);

  val = (float)((int)dummy) * varMultiplier;
  return val;
}