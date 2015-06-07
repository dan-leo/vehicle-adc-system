#ifndef ADCPIV3_H
#define ADCPIV3_H

// define adc chips addresses and channel modes
#define ADC_1     0x68
#define ADC_2     0x69
#define ADC_CHANNEL1  0x9C
#define ADC_CHANNEL2  0xBC
#define ADC_CHANNEL3  0xDC
#define ADC_CHANNEL4  0xFC

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

extern const float varDivisior; // from pdf sheet on adc addresses and config for 18 bit mode
float varMultiplier;

float getadc (int chn);

#endif /* ADCPIV3_H */