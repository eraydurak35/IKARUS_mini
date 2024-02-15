#ifndef SPI_H
#define SPI_H

#include <stdio.h>

#define SCL_PIN  GPIO_NUM_18
#define MISO_PIN GPIO_NUM_19
#define MOSI_PIN GPIO_NUM_23
#define ICM42688P_CS_PIN 5
#define BMP390_CS_PIN 13
#define SPI_FREQ 10000000



void spi_master_init(uint8_t MISO, uint8_t MOSI, uint8_t SCL);
void spi_read_ICM42688P(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
void spi_write_ICM42688P(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);

void spi_read_BMP390(uint8_t reg_addr, uint8_t *reg_data, uint32_t len);
void spi_write_BMP390(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len);


#endif