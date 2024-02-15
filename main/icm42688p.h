#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdio.h>
#include "driver/spi_master.h"
#include "spi.h"

#define X 0
#define Y 1
#define Z 2

#define GYRO_CONFIG0 0x4F
#define ACCEL_CONFIG0 0x50
#define PWR_MGMT0 0x4E

#define GY_FS_2000 0
#define GY_FS_1000 1
#define GY_FS_500 2
#define GY_FS_250 3
#define GY_FS_125 4
#define GY_FS_62_5 5
#define GY_FS_31_25 6
#define GY_FS_15_625 7

#define GY_ODR_32KHZ 1
#define GY_ODR_16KHZ 2
#define GY_ODR_8KHZ 3
#define GY_ODR_4KHZ 4
#define GY_ODR_2KHZ 5
#define GY_ODR_1KHZ 6
#define GY_ODR_200HZ 7
#define GY_ODR_100HZ 8
#define GY_ODR_50HZ 9
#define GY_ODR_25HZ 10
#define GY_ODR_12_5HZ 11
#define GY_ODR_500HZ 15


#define AC_FS_16G 0
#define AC_FS_8G 1
#define AC_FS_4G 2
#define AC_FS_2G 3

#define AC_ODR_32KHZ 1
#define AC_ODR_16KHZ 2
#define AC_ODR_8KHZ 3
#define AC_ODR_4KHZ 4
#define AC_ODR_2KHZ 5
#define AC_ODR_1KHZ 6
#define AC_ODR_200HZ 7
#define AC_ODR_100HZ 8
#define AC_ODR_50HZ 9
#define AC_ODR_25HZ 10


#define AC_MODE_LOW_NOISE 3
#define AC_MODE_LOW_PWR 2
#define AC_MODE_OFF 0

#define GY_MODE_STDBY 1
#define GY_MODE_LOW_NOISE 3
#define GY_MODE_OFF 0

#define TEMP_SENS_ENABLE 0
#define TEMP_SENS_DISABLE 1

#define TEMP_DATA1 0x1D

#define GYR_GAIN 0.0304878f
#define ACC_GAIN 0.0023940455f

typedef struct
{
    float gyro_dps[3];
    float accel_ms2[3];
    int16_t temp_mC;
    float gyro_bias_dps[3];
} icm42688p_t;

void icm42688p_setup();
void icm42688p_read(icm42688p_t *imu);

#endif