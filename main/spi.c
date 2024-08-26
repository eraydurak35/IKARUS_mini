#include <stdio.h>
#include "driver/spi_master.h"
#include "spi.h"
#include <string.h>

void spi_master_init(uint8_t miso_pin_num, uint8_t mosi_pin_num, uint8_t clk_pin_num)
{
    // SPI bus tanımı
    spi_bus_config_t buscfg={
        .miso_io_num=miso_pin_num,
        .mosi_io_num=mosi_pin_num,
        .sclk_io_num=clk_pin_num,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
}

void spi_add_device_to_bus(spi_device_handle_t *dev, int cs_pin_num, int speed)
{
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=speed,
        .mode=0,
        .spics_io_num=cs_pin_num,
        .queue_size=1
    };

    spi_bus_add_device(SPI2_HOST, &devcfg, dev);
}

uint8_t spi_read_byte(spi_device_handle_t dev, uint8_t reg_addr)
{
    reg_addr = reg_addr | 0x80;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    t.rxlength = 8;
    t.tx_buffer = &reg_addr;
    uint8_t buf[2];
    memset(buf, 0, 2);
    t.rx_buffer = buf;
    spi_device_polling_transmit(dev, &t);
    return buf[1];
}

void spi_read_bytes(spi_device_handle_t dev, uint8_t reg_addr, uint8_t *txrx_buffer, size_t len)
{
    reg_addr = reg_addr | 0x80;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8+8*len;
    t.rxlength = 8*len;
    t.tx_buffer = &reg_addr;
    uint8_t buf[len+1];
    memset(buf, 0, len+1);
    t.rx_buffer = buf;
    spi_device_polling_transmit(dev, &t);
    memmove(txrx_buffer, &buf[1], len);
}

void spi_write_byte(spi_device_handle_t dev, uint8_t reg_addr, uint8_t data)
{
    reg_addr = reg_addr & 0x7F;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 16;
    uint8_t buf[2];
    buf[0] = reg_addr;
    buf[1] = data;
    t.tx_buffer = buf;
    spi_device_polling_transmit(dev, &t);
}