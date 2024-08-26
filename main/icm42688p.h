#ifndef ICM42688P_H
#define ICM42688P_H

#include <stdio.h>
#include "driver/spi_master.h"
#include "spi.h"
#include "typedefs.h"

#define GYRO_CONFIG0 0x4F
#define ACCEL_CONFIG0 0x50
#define PWR_MGMT0 0x4E

#define GY_FS_2000 (0 << 5)
#define GY_FS_1000 (1 << 5)
#define GY_FS_500 (2 << 5)
#define GY_FS_250 (3 << 5)
#define GY_FS_125 (4 << 5)
#define GY_FS_62_5 (5 << 5)
#define GY_FS_31_25 (6 << 5)
#define GY_FS_15_625 (7 << 5)

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


#define AC_FS_16G (0 << 5)
#define AC_FS_8G (1 << 5)
#define AC_FS_4G (2 << 5)
#define AC_FS_2G (3 << 5)

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

#define GYR_GAIN 0.06103515625f
#define ACC_GAIN 0.0047880859375f

#define ICM426XX_RA_REG_BANK_SEL                    0x76
#define ICM426XX_BANK_SELECT0                       0x00
#define ICM426XX_BANK_SELECT1                       0x01
#define ICM426XX_BANK_SELECT2                       0x02
#define ICM426XX_BANK_SELECT3                       0x03
#define ICM426XX_BANK_SELECT4                       0x04

// Fix for stalls in gyro output. See https://github.com/ArduPilot/ardupilot/pull/25332
#define ICM426XX_INTF_CONFIG1                       0x4D
#define ICM426XX_INTF_CONFIG1_AFSR_MASK             0xC0
#define ICM426XX_INTF_CONFIG1_AFSR_DISABLE          0x40

#define ICM426XX_RA_PWR_MGMT0                       0x4E  // User Bank 0
#define ICM426XX_PWR_MGMT0_ACCEL_MODE_LN            (3 << 0)
#define ICM426XX_PWR_MGMT0_GYRO_MODE_LN             (3 << 2)
#define ICM426XX_PWR_MGMT0_GYRO_ACCEL_MODE_OFF      ((0 << 0) | (0 << 2))
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_OFF         (0 << 5)
#define ICM426XX_PWR_MGMT0_TEMP_DISABLE_ON          (1 << 5)

#define ICM426XX_RA_GYRO_CONFIG0                    0x4F
#define ICM426XX_RA_ACCEL_CONFIG0                   0x50

// --- Registers for gyro and acc Anti-Alias Filter ---------
#define ICM426XX_RA_GYRO_CONFIG_STATIC3             0x0C  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC4             0x0D  // User Bank 1
#define ICM426XX_RA_GYRO_CONFIG_STATIC5             0x0E  // User Bank 1
#define ICM426XX_RA_ACCEL_CONFIG_STATIC2            0x03  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC3            0x04  // User Bank 2
#define ICM426XX_RA_ACCEL_CONFIG_STATIC4            0x05  // User Bank 2
// --- Register & setting for gyro and acc UI Filter --------
#define ICM426XX_RA_GYRO_ACCEL_CONFIG0              0x52  // User Bank 0
#define ICM426XX_ACCEL_UI_FILT_BW_LOW_LATENCY       (15 << 4) 
#define ICM426XX_GYRO_UI_FILT_BW_LOW_LATENCY        (15 << 0)
// ----------------------------------------------------------

#define ICM426XX_RA_GYRO_DATA_X1                    0x25  // User Bank 0
#define ICM426XX_RA_ACCEL_DATA_X1                   0x1F  // User Bank 0

#define ICM426XX_RA_INT_CONFIG                      0x14  // User Bank 0
#define ICM426XX_INT1_MODE_PULSED                   (0 << 2)
#define ICM426XX_INT1_MODE_LATCHED                  (1 << 2)
#define ICM426XX_INT1_DRIVE_CIRCUIT_OD              (0 << 1)
#define ICM426XX_INT1_DRIVE_CIRCUIT_PP              (1 << 1)
#define ICM426XX_INT1_POLARITY_ACTIVE_LOW           (0 << 0)
#define ICM426XX_INT1_POLARITY_ACTIVE_HIGH          (1 << 0)

#define ICM426XX_RA_INT_CONFIG0                     0x63  // User Bank 0
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR           ((0 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_DUPLICATE ((0 << 5) || (0 << 4)) // duplicate settings in datasheet, Rev 1.2.
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_F1BR          ((1 << 5) || (0 << 4))
#define ICM426XX_UI_DRDY_INT_CLEAR_ON_SBR_AND_F1BR  ((1 << 5) || (1 << 4))

#define ICM426XX_RA_INT_CONFIG1                     0x64   // User Bank 0
#define ICM426XX_INT_ASYNC_RESET_BIT                4
#define ICM426XX_INT_TDEASSERT_DISABLE_BIT          5
#define ICM426XX_INT_TDEASSERT_ENABLED              (0 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TDEASSERT_DISABLED             (1 << ICM426XX_INT_TDEASSERT_DISABLE_BIT)
#define ICM426XX_INT_TPULSE_DURATION_BIT            6
#define ICM426XX_INT_TPULSE_DURATION_100            (0 << ICM426XX_INT_TPULSE_DURATION_BIT)
#define ICM426XX_INT_TPULSE_DURATION_8              (1 << ICM426XX_INT_TPULSE_DURATION_BIT)

#define ICM426XX_RA_INT_SOURCE0                     0x65  // User Bank 0
#define ICM426XX_UI_DRDY_INT1_EN_DISABLED           (0 << 3)
#define ICM426XX_UI_DRDY_INT1_EN_ENABLED            (1 << 3)


void icm42688p_setup(calibration_t *acc_cal);
void icm42688p_read(imu_t *imu);

#endif