#ifndef SPI_H
#define SPI_H

#include <stdio.h>
#include "driver/spi_master.h"

#define CLK_PIN  13
#define MISO_PIN 11
#define MOSI_PIN 10
#define ICM42688P_CS_PIN 12
#define BMP390_CS_PIN 14
#define SPI_FREQ 10000000


void spi_master_init(uint8_t miso_pin_num, uint8_t mosi_pin_num, uint8_t clk_pin_num);
void spi_add_device_to_bus(spi_device_handle_t *dev, int cs_pin_num, int speed);
uint8_t spi_read_byte(spi_device_handle_t dev, uint8_t reg_addr);
void spi_read_bytes(spi_device_handle_t dev, uint8_t reg_addr, uint8_t *txrx_buffer, size_t len);
void spi_write_byte(spi_device_handle_t dev, uint8_t reg_addr, uint8_t data);

#endif