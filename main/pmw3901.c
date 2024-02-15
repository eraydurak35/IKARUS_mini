#include <stdio.h>
#include "pmw3901.h"
#include "uart.h"

static uint8_t data_frame_buff[8] = {0};
static uint8_t is_pmw3901_header_found = 0;
static uint8_t pwm3901_byte_counter = 0;


void parse_pmw3901_data(pmw3901_t *pmw, uart_data_t *buff)
{

  for (uint8_t i = 0; i < buff->lenght; i++)
  {

    if (is_pmw3901_header_found == 1)
    {
      data_frame_buff[pwm3901_byte_counter] = buff->data[i];
      pwm3901_byte_counter++;

      if (pwm3901_byte_counter >= 8)
      {
        pwm3901_byte_counter = 0;
        is_pmw3901_header_found = 0;

        uint8_t checksum = data_frame_buff[1] + data_frame_buff[2] + data_frame_buff[3] + data_frame_buff[4];
        if (checksum == data_frame_buff[5] && data_frame_buff[7] == PMW3901_FOOTER)
        {
          pmw->raw_x_cpi = data_frame_buff[2] << 8 | data_frame_buff[1];
          pmw->raw_y_cpi = data_frame_buff[4] << 8 | data_frame_buff[3];
          // Simple low pass filter
          pmw->quality += ((data_frame_buff[6] / 2.0f) - pmw->quality) * 0.1f;
          //flow_tilt_compansation();
        }
      }
    }
    else if (buff->data[i] == PMW3901_HEADER)
    {
      is_pmw3901_header_found = 1;
    }
  }
}