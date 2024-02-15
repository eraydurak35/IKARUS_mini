#include <stdio.h>
#include "hmc5883l.h"
#include "i2c.h"

static void parse_hmc5883l_data(hmc5883l_t *hmc, uint8_t *buffer);
static void get_calibrated_result(hmc5883l_t *hmc);

static const float mag_bias[3] = 
{
    336.194593f, 448.029001f, -78.888176f
};
static const float mag_cal_matrix[3][3] = 
{
    {0.974868f, 0.012356f, 0.012350f},
    {0.012356f, 0.946851f, -0.010228f},
    {0.012350f, -0.010228f, 1.055808f}
};


void hmc5883l_setup()
{
    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, HMC5883L_ADDR, CONFIG_REG_A, AVERAGE_8 << 5 | ODR_75HZ << 2 | MODE_NORMAL_HMC);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, HMC5883L_ADDR, CONFIG_REG_B, RES_5_6_GAUSS << 5);

    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_write_data(I2C_NUM_0, HMC5883L_ADDR, MODE_REG, HS_I2C_ENABLE << 7 | MEAS_CONTINUOUS);
}

void hmc5883l_read(hmc5883l_t *hmc)
{
    static uint8_t buff[6] = {0};
    i2c_read_data(I2C_NUM_0, HMC5883L_ADDR, X_MSB, buff, 6);
    parse_hmc5883l_data(hmc, buff);
    get_calibrated_result(hmc);
}

static void parse_hmc5883l_data(hmc5883l_t *hmc, uint8_t *buffer)
{   
    // X Z Y order
    hmc->axis[0] = (float)((int16_t)(buffer[0] << 8 | buffer[1]));
    hmc->axis[2] = (float)((int16_t)(buffer[2] << 8 | buffer[3]));
    hmc->axis[1] = (float)((int16_t)(buffer[4] << 8 | buffer[5]));
}

static void get_calibrated_result(hmc5883l_t *hmc)
{

    static float temp_x = 0;
    static float temp_y = 0;
    static float temp_z = 0; 

    temp_x = hmc->axis[0] - mag_bias[0];
    temp_y = hmc->axis[1] - mag_bias[1];
    temp_z = hmc->axis[2] - mag_bias[2];

    hmc->axis[0] = mag_cal_matrix[0][0] * temp_x + mag_cal_matrix[0][1] * temp_y + mag_cal_matrix[0][2] * temp_z;
    hmc->axis[1] = mag_cal_matrix[1][0] * temp_x + mag_cal_matrix[1][1] * temp_y + mag_cal_matrix[1][2] * temp_z;
    hmc->axis[2] = mag_cal_matrix[2][0] * temp_x + mag_cal_matrix[2][1] * temp_y + mag_cal_matrix[2][2] * temp_z;
}