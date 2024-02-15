#ifndef BMP390_H
#define BMP390_H

#include <stdio.h>


#define BMP390_ADDR 0x77
#define CALIB_START_REG 0x31
#define CONFIG_BMP390 0x1F
#define ODR_REG 0x1D
#define OSR_REG 0x1C
#define PWR_CTRL 0x1B
#define DATA_0 0x04

#define IIR_FILT_0 0
#define IIR_FILT_1 1
#define IIR_FILT_3 2
#define IIR_FILT_7 3
#define IIR_FILT_15 4
#define IIR_FILT_31 5
#define IIR_FILT_63 6
#define IIR_FILT_127 7

#define ODR_200HZ 0
#define ODR_100HZ 1
#define ODR_50HZ 2
#define ODR_25HZ 3
#define ODR_12_5HZ 4

#define OSR_PRES_NO_OSR 0
#define OSR_PRES_2X 1
#define OSR_PRES_4X 2
#define OSR_PRES_8X 3
#define OSR_PRES_16X 4
#define OSR_PRES_32X 5

#define OSR_TEMP_NO_OSR 0
#define OSR_TEMP_2X 1
#define OSR_TEMP_4X 2
#define OSR_TEMP_8X 3
#define OSR_TEMP_16X 4
#define OSR_TEMP_32X 5

#define MODE_SLEEP 0
#define MODE_FORCED 1
#define MODE_NORMAL 3

#define TEMP_ON 1
#define TEMP_OFF 0
#define PRESS_ON 1
#define PRESS_OFF 0




typedef struct
{
  float par_t1;
  float par_t2;
  float par_t3;
  float par_p1;
  float par_p2;
  float par_p3;
  float par_p4;
  float par_p5;
  float par_p6;
  float par_p7;
  float par_p8;
  float par_p9;
  float par_p10;
  float par_p11;
} bmp390_calib_t;

typedef struct
{
    int32_t pressADC;
    int32_t tempADC;
    float temp;
    float press;
    float gnd_press;
    float altitude_m;
    float velocity_ms;
    float init_temp;
} bmp390_t;

void bmp390_setup_i2c();
void bmp390_setup_spi();
void bmp390_read_i2c(bmp390_t *bmp);
void bmp390_read_spi(bmp390_t *bmp);
void baro_set_ground_pressure(bmp390_t *baro);
void baro_get_altitude_velocity(bmp390_t *baro);

#endif