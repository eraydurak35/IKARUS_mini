#ifndef HMC5883L_H
#define HMC5883L_H

#include <stdio.h>
#include "typedefs.h"

#define HMC5883L_ADDR 0x1E
#define CONFIG_REG_A 0x0
#define CONFIG_REG_B 0x01
#define MODE_REG 0x02

#define AVERAGE_1 0
#define AVERAGE_2 1
#define AVERAGE_4 2
#define AVERAGE_8 3

#define ODR_0_75HZ 0
#define ODR_1_5HZ 1
#define ODR_3HZ 2
#define ODR_7_5HZ 3
#define ODR_15HZ 4
#define ODR_30HZ 5
#define ODR_75HZ 6

#define MODE_NORMAL_HMC 0
#define MODE_POS_BIAS_HMC 1
#define MODE_NEG_BIAS_HMC 2

// 0 00 1 Average   000 0.75Hz DOR    00 Normal
// 0 01 2 Average   001 1.5Hz  DOR    01 Pozitive Bias Mode
// 0 10 4 Average   010 3Hz    DOR    10 Negative Bias Mode
// 0 11 8 Average   011 7.5Hz  DOR
//                  100 15Hz   DOR
//                  101 30Hz   DOR
//                  110 75Hz   DOR

#define RES_0_8_GAUSS 0
#define RES_1_3_GAUSS 1
#define RES_1_9_GAUSS 2
#define RES_2_5_GAUSS 3
#define RES_4_0_GAUSS 4
#define RES_4_7_GAUSS 5
#define RES_5_6_GAUSS 6
#define RES_8_1_GAUSS 7

// 000 0.88 Gauss   00000
// 001 1.3  Gauss   00000
// 010 1.9  Gauss   00000
// 011 2.5  Gauss   00000
// 100 4    Gauss   00000
// 101 4.7  Gauss   00000
// 110 5.6  Gauss   00000
// 111 8.1  Gauss   00000

#define HS_I2C_ENABLE 1
#define HS_I2C_DISABLE 0

#define MEAS_CONTINUOUS 0
#define MEAS_SINGLE 1

// 0 Disable HS(3400kHz) I2C  00000   00 Continuous Measurement
// 1 Enable HS(3400kHz) I2C   00000   01 Single Measurement

#define X_MSB 0x03

void hmc5883l_setup(float *mg);
void hmc5883l_read(magnetometer_t *hmc);

#endif