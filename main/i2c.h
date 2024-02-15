#ifndef I2C_H
#define I2C_H

#include "driver/i2c.h"
#include <stdio.h>

#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1

#define SDA1 GPIO_NUM_21
#define SCL1 GPIO_NUM_22

void i2c_master_init(i2c_port_t port_num, uint8_t SDA, uint8_t SCL, uint32_t speed, uint8_t gpio_pull_up);
void i2c_read_data(i2c_port_t port_num, uint8_t slave_addr, uint8_t register_read_from, uint8_t* buffer, size_t size);
void i2c_write_data(i2c_port_t port_num, uint8_t slave_addr, uint8_t reg, uint8_t value);

#endif