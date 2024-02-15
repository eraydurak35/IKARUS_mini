#ifndef PMW3901_H
#define PMW3901_H
#include <stdio.h>
#include "uart.h"

//=================================//
//   PMW3901U serial description   //
//=================================//
//  byte0: header (0xFE)           //
//  byte1: data size (fixed 0x04)  //
//  byte2: x-motion low byte       //
//  byte3: x-motion high byte      //
//  byte4: y-motion low byte       //
//  byte5: y-motion high byte      //
//  byte6: checksum                //
//  byte7: surface quality         //
//  byte8: footer (0xAA)?(0XBB)    //
//  baud rate: 19200 bps           //
//  data bits: 8                   //
//  stop bits 1                    //
//  parity: no parity              //
//=================================//

#define PMW3901_HEADER 0xFE
#define PMW3901_FOOTER 0xBB//0xAA

typedef struct
{
  int16_t raw_x_cpi, raw_y_cpi;
  float filt_x_cpi, filt_y_cpi;
  float velocity_x_ms, velocity_y_ms;
  float quality;
} pmw3901_t;

void parse_pmw3901_data(pmw3901_t *pmw, uart_data_t *buff);


#endif