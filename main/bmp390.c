#include <stdio.h>
#include <math.h>
#include "bmp390.h"
#include "i2c.h"
#include "spi.h"

static bmp390_calib_t calib;

static void parse_bmp390_calib_bytes(uint8_t *buffer);
static float bmp390_compans_temp(uint32_t tempADC);
static float bmp390_compans_press(uint32_t pressADC, float temp);
static void parse_bmp390_data_bytes(bmp390_t *bmp, uint8_t *buff);

void bmp390_setup_i2c()
{
  uint8_t buff[21] = {0};
  i2c_read_data(I2C_NUM_1, BMP390_ADDR, CALIB_START_REG, buff, 21);

  parse_bmp390_calib_bytes(buff);

  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_data(I2C_NUM_1, BMP390_ADDR, CONFIG_BMP390, IIR_FILT_7 << 1);

  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_data(I2C_NUM_1, BMP390_ADDR, ODR_REG, ODR_50HZ);

  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_data(I2C_NUM_1, BMP390_ADDR, OSR_REG, OSR_TEMP_NO_OSR << 3 | OSR_PRES_8X);

  vTaskDelay(10 / portTICK_PERIOD_MS);
  i2c_write_data(I2C_NUM_1, BMP390_ADDR, PWR_CTRL, MODE_NORMAL << 4 | TEMP_ON << 1 | PRESS_ON);
}

void bmp390_setup_spi()
{

  uint8_t buff[22] = {0};

  vTaskDelay(100 / portTICK_PERIOD_MS);
  spi_read_BMP390(CALIB_START_REG, buff, 22);

  // FIRST BYTE IS DUMMY (DISCARTED)
  parse_bmp390_calib_bytes(buff + 1);

  uint8_t data[1];
  vTaskDelay(10 / portTICK_PERIOD_MS);

  data[0] = IIR_FILT_7 << 1;
  // data[0] = 0x06;
  spi_write_BMP390(CONFIG_BMP390, data, 1);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  data[0] = ODR_50HZ;
  // data[0] = 0x02;
  spi_write_BMP390(ODR_REG, data, 1);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  data[0] = OSR_TEMP_NO_OSR << 3 | OSR_PRES_8X;
  // data[0] = 0x03;
  spi_write_BMP390(OSR_REG, data, 1);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  data[0] = MODE_NORMAL << 4 | TEMP_ON << 1 | PRESS_ON;
  // data[0] = 0x33;
  spi_write_BMP390(PWR_CTRL, data, 1);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

static void parse_bmp390_calib_bytes(uint8_t *buffer)
{

  uint16_t NVM_PAR_T1 = buffer[0] | buffer[1] << 8;
  uint16_t NVM_PAR_T2 = buffer[2] | buffer[3] << 8;
  int8_t NVM_PAR_T3 = buffer[4];
  int16_t NVM_PAR_P1 = buffer[5] | buffer[6] << 8;
  int16_t NVM_PAR_P2 = buffer[7] | buffer[8] << 8;
  int8_t NVM_PAR_P3 = buffer[9];
  int8_t NVM_PAR_P4 = buffer[10];
  uint16_t NVM_PAR_P5 = buffer[11] | buffer[12] << 8;
  uint16_t NVM_PAR_P6 = buffer[13] | buffer[14] << 8;
  int8_t NVM_PAR_P7 = buffer[15];
  int8_t NVM_PAR_P8 = buffer[16];
  int16_t NVM_PAR_P9 = buffer[17] | buffer[18] << 8;
  int8_t NVM_PAR_P10 = buffer[19];
  int8_t NVM_PAR_P11 = buffer[20];

  calib.par_t1 = (float)NVM_PAR_T1 * 256.0f;
  calib.par_t2 = (float)NVM_PAR_T2 / 1073741824.0f;
  calib.par_t3 = (float)NVM_PAR_T3 / 281474976710656.0f;
  calib.par_p1 = ((float)NVM_PAR_P1 - 16384) / 1048576.0f;
  calib.par_p2 = ((float)NVM_PAR_P2 - 16384) / 536870912.0f;
  calib.par_p3 = (float)NVM_PAR_P3 / 4294967296.0f;
  calib.par_p4 = (float)NVM_PAR_P4 / 137438953472.0f;
  calib.par_p5 = (float)NVM_PAR_P5 * 8.0f;
  calib.par_p6 = (float)NVM_PAR_P6 / 64.0f;
  calib.par_p7 = (float)NVM_PAR_P7 / 256.0f;
  calib.par_p8 = (float)NVM_PAR_P8 / 32768.0f;
  calib.par_p9 = (float)NVM_PAR_P9 / 281474976710656.0f;
  calib.par_p10 = (float)NVM_PAR_P10 / 281474976710656.0f;
  calib.par_p11 = (float)NVM_PAR_P11 / 36893488147419103232.0f;
}

void bmp390_read_i2c(bmp390_t *bmp)
{
  static uint8_t buff[6] = {0};
  i2c_read_data(I2C_NUM_1, BMP390_ADDR, DATA_0, buff, 6);
  parse_bmp390_data_bytes(bmp, buff);
}

void bmp390_read_spi(bmp390_t *bmp)
{
  static uint8_t buff[7] = {0};
  spi_read_BMP390(DATA_0, buff, 7);
  parse_bmp390_data_bytes(bmp, buff + 1);
}

static void parse_bmp390_data_bytes(bmp390_t *bmp, uint8_t *buff)
{
  bmp->pressADC = buff[0] | buff[1] << 8 | buff[2] << 16;
  bmp->tempADC = buff[3] | buff[4] << 8 | buff[5] << 16;
  bmp->temp = bmp390_compans_temp(bmp->tempADC);
  bmp->press = bmp390_compans_press(bmp->pressADC, bmp->temp);
}

static float bmp390_compans_temp(uint32_t tempADC)
{
  static float partial_data1;
  static float partial_data2;
  partial_data1 = (float)(tempADC - calib.par_t1);
  partial_data2 = (float)(partial_data1 * calib.par_t2);
  return (partial_data2 + (partial_data1 * partial_data1) * calib.par_t3);
}

static float bmp390_compans_press(uint32_t pressADC, float temp)
{
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;
  float temp_square = temp * temp;
  float temp_cube = temp_square * temp;
  partial_data1 = calib.par_p6 * temp;
  partial_data2 = calib.par_p7 * temp_square;
  partial_data3 = calib.par_p8 * temp_cube;
  partial_out1 = calib.par_p5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = calib.par_p2 * temp;
  partial_data2 = calib.par_p3 * temp_square;
  partial_data3 = calib.par_p4 * temp_cube;
  partial_out2 = (float)pressADC * (calib.par_p1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = (float)pressADC * (float)pressADC;
  partial_data2 = calib.par_p9 + calib.par_p10 * temp;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((float)pressADC * (float)pressADC * (float)pressADC) * calib.par_p11;
  return ((partial_out1 + partial_out2 + partial_data4) / 100.0f);
}



void baro_set_ground_pressure(bmp390_t *baro)
{
  float press_accumulator = 0;
  for (uint8_t i = 0; i < 20; i++)
  {
    bmp390_read_spi(baro);
    press_accumulator += baro->press;
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
  baro->gnd_press = press_accumulator / 20.0;
  baro->init_temp = baro->temp;
}

void baro_get_altitude_velocity(bmp390_t *baro)
{
  static uint8_t first_time = 1;
  static float prev_altitude_m = 0.0f;

  baro->altitude_m = 44330.0f * (1.0f - powf(baro->press / baro->gnd_press, 0.19029f));
  baro->altitude_m -= (baro->temp - baro->init_temp) * 0.0453f;

  if (first_time == 1)
  {
    prev_altitude_m = baro->altitude_m;
    first_time = 0;
  }
  baro->velocity_ms = (baro->altitude_m - prev_altitude_m) / 0.02f;
  prev_altitude_m = baro->altitude_m;
}