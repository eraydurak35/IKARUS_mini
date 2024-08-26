#ifndef QMC5883L_H
#define QMC5883L_H

#include <stdio.h>
#include "typedefs.h"

#define QMC5883L_ADDR 0x0D
#define CTRL_REG_1 0x09
#define CTRL_REG_2 0x0A
#define SET_RESET_REG 0x0B
#define STATUS_REG 0x06
#define OUTPUT_X_REG 0x00

#define MODE_STDBY 0
#define MODE_CONTINUOUS 1

#define ODR_10_HZ (0 << 2)
#define ODR_50_HZ (1 << 2)
#define ODR_100_HZ (2 << 2)
#define ODR_200_HZ (3 << 2)

#define SCALE_2_GAUSS (0 << 4)
#define SCALE_8_GAUSS (1 << 4)

#define OSR_512 (0 << 6)
#define OSR_256 (1 << 6)
#define OSR_128 (2 << 6)
#define OSR_64 (3 << 6)


void qmc5883l_setup(calibration_t *mg_cal);
void qmc5883l_read(magnetometer_t *qmc, uint16_t throttle);


#endif