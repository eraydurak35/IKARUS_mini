#include <stdio.h>
#include "uart.h"
#include "driver/uart.h"



void uart_begin(uart_port_t port, int baud_rate, uint8_t tx_pin, uint8_t rx_pin, uart_parity_t parity)
{
    uart_config_t uart_config = 
    {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = parity,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(port, &uart_config);
    uart_set_pin(port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(port, BUFF_SIZE, BUFF_SIZE, 0, NULL, 0);
}



void uart_read(uart_port_t port, uart_data_t *uart, uint16_t timeout_ms)
{
    uint8_t len = uart_read_bytes(port, uart->data, BUFF_SIZE, timeout_ms);
    uart->lenght = len;

    //printf("%d\n", len);

/*     for (uint8_t i = 0; i < len; i++)
    {
        printf("%x  ", uart->data[i]);
    }
    printf("\n"); */
}