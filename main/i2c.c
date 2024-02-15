#include <stdio.h>
#include "i2c.h"
#include "driver/i2c.h"


void i2c_master_init(i2c_port_t port_num, uint8_t SDA, uint8_t SCL, uint32_t speed, uint8_t gpio_pull_up)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA,
        .scl_io_num = SCL,
        .sda_pullup_en = gpio_pull_up,
        .scl_pullup_en = gpio_pull_up,
        .master.clk_speed = speed
    };
    i2c_param_config(port_num, &i2c_config);
    i2c_driver_install(port_num, I2C_MODE_MASTER, 0, 0, 0);
}


void i2c_read_data(i2c_port_t port_num, uint8_t slave_addr, uint8_t register_read_from, uint8_t* buffer, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_read_from, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_addr << 1) | I2C_MASTER_READ, true);
    if (size > 1) i2c_master_read(cmd, buffer, size - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buffer + size - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(port_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}



void i2c_write_data(i2c_port_t port_num, uint8_t slave_addr, uint8_t reg, uint8_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, slave_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(port_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
