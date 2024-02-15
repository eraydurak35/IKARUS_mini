#include <stdio.h>
#include "icm42688p.h"
#include "spi.h"
#include "driver/spi_master.h"
#include "freertos/task.h"


const float accel_calib_matrix[3][3] =
{
  {0.998717f, -0.001619f, 0.000346f},
  {-0.001619f, 0.999985f, 0.000127f},
  {0.000346f, 0.000127f, 1.000793f}
};
const float accel_bias_ms2[3] = {-0.019807f, -0.008603f, 0.071821f};

static void parse_icm42688p_data(icm42688p_t *imu, uint8_t *buff);
static void getCalibratedResults(icm42688p_t *imu);
void icm42688p_setup()
{
    uint8_t data[1];
    
    vTaskDelay(200 / portTICK_PERIOD_MS);
    data[0] = GY_FS_1000 << 5 | GY_ODR_2KHZ; // 0X25
    spi_write_ICM42688P(GYRO_CONFIG0, data, 1);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    data[0] = AC_FS_8G << 5 | AC_ODR_2KHZ; // 0X25
    spi_write_ICM42688P(ACCEL_CONFIG0, data, 1);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    data[0] = TEMP_SENS_ENABLE << 5 | GY_MODE_LOW_NOISE << 2 | AC_MODE_LOW_NOISE; // 0xF
    spi_write_ICM42688P(PWR_MGMT0, data, 1);

    vTaskDelay(10 / portTICK_PERIOD_MS);
}


void icm42688p_read(icm42688p_t *imu)
{
    uint8_t data_read[14];
    spi_read_ICM42688P(TEMP_DATA1, data_read, 14);
    parse_icm42688p_data(imu, data_read);
    getCalibratedResults(imu);
}

static void parse_icm42688p_data(icm42688p_t *imu, uint8_t *buff)
{
    imu->temp_mC = ((int16_t)(buff[0] << 8 | buff[1]) / 1.3248f) + 2500;
    imu->accel_ms2[X] = (int16_t)(buff[2] << 8 | buff[3]) * ACC_GAIN;
    imu->accel_ms2[Y] = (int16_t)(buff[4] << 8 | buff[5]) * ACC_GAIN;
    imu->accel_ms2[Z] = (int16_t)(buff[6] << 8 | buff[7]) * ACC_GAIN;

    imu->gyro_dps[X] = (int16_t)(buff[8] << 8 | buff[9]) * GYR_GAIN;
    imu->gyro_dps[Y] = (int16_t)(buff[10] << 8 | buff[11]) * GYR_GAIN;
    imu->gyro_dps[Z] = (int16_t)(buff[12] << 8 | buff[13]) * GYR_GAIN;
}

static void getCalibratedResults(icm42688p_t *imu)
{
    imu->gyro_dps[X] -= imu->gyro_bias_dps[X];
    imu->gyro_dps[Y] -= imu->gyro_bias_dps[Y];
    imu->gyro_dps[Z] -= imu->gyro_bias_dps[Z];

    static float temp_x = 0.0f;
    static float temp_y = 0.0f;
    static float temp_z = 0.0f;

    temp_x = imu->accel_ms2[X] - accel_bias_ms2[X];
    temp_y = imu->accel_ms2[Y] - accel_bias_ms2[Y];
    temp_z = imu->accel_ms2[Z] - accel_bias_ms2[Z];

    imu->accel_ms2[X] = accel_calib_matrix[0][0] * temp_x + accel_calib_matrix[0][1] * temp_y + accel_calib_matrix[0][2] * temp_z;
    imu->accel_ms2[Y] = accel_calib_matrix[1][0] * temp_x + accel_calib_matrix[1][1] * temp_y + accel_calib_matrix[1][2] * temp_z;
    imu->accel_ms2[Z] = accel_calib_matrix[2][0] * temp_x + accel_calib_matrix[2][1] * temp_y + accel_calib_matrix[2][2] * temp_z;
}