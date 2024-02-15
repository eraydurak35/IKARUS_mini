#include <stdio.h>
#include "driver/spi_master.h"
#include "spi.h"
#include <string.h>


spi_device_handle_t ICM42688P_spi_handle;
spi_device_handle_t BMP390_spi_handle;

static void spi_add_ICM42688P_to_bus(int speed, uint8_t cs_pin);
static void spi_add_BMP390_to_bus(int speed, uint8_t cs_pin);

void spi_master_init(uint8_t MISO, uint8_t MOSI, uint8_t SCL)
{
    // SPI bus tanımı
    spi_bus_config_t buscfg={
        .miso_io_num=MISO,
        .mosi_io_num=MOSI,
        .sclk_io_num=SCL,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };

    spi_bus_initialize(VSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);

    spi_add_ICM42688P_to_bus(SPI_FREQ, ICM42688P_CS_PIN);
    spi_add_BMP390_to_bus(SPI_FREQ, BMP390_CS_PIN);
}



static void spi_add_ICM42688P_to_bus(int speed, uint8_t cs_pin)
{
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=speed,
        .mode=0,
        .spics_io_num=cs_pin,
        .queue_size=1
    };

    spi_bus_add_device(VSPI_HOST, &devcfg, &ICM42688P_spi_handle);
}


static void spi_add_BMP390_to_bus(int speed, uint8_t cs_pin)
{
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=speed,
        .mode=0,
        .spics_io_num=cs_pin,
        .queue_size=1
    };

    spi_bus_add_device(VSPI_HOST, &devcfg, &BMP390_spi_handle);
}


void spi_read_ICM42688P(uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
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
    spi_device_polling_transmit(ICM42688P_spi_handle, &t);
    memmove(reg_data, &buf[1], len);
}



void spi_write_ICM42688P(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    reg_addr = reg_addr & 0x7F;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8*len+8;
    uint8_t buf[len+1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);
    t.tx_buffer = buf;
    spi_device_polling_transmit(ICM42688P_spi_handle, &t);
}



























void spi_read_BMP390(uint8_t reg_addr, uint8_t *reg_data, uint32_t len)
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
    spi_device_polling_transmit(BMP390_spi_handle, &t);
    memmove(reg_data, &buf[1], len);
}



void spi_write_BMP390(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len)
{
    reg_addr = reg_addr & 0x7F;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8*len+8;
    uint8_t buf[len+1];
    buf[0] = reg_addr;
    memcpy(&buf[1], reg_data, len);
    t.tx_buffer = buf;
    spi_device_polling_transmit(BMP390_spi_handle, &t);
}