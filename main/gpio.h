#ifndef GPIO_H
#define GPIO_H
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"


#define VSENS_CHANNEL ADC_CHANNEL_4 //GPIO_NUM_5

#define MOT1_PIN 16
#define MOT2_PIN 4
#define MOT3_PIN 38
#define MOT4_PIN 1
#define LED_PIN 2
#define BUTTON_PIN 0


void gpio_configure();
void set_throttle(uint16_t mot1_thr, uint16_t mot2_thr, uint16_t mot3_thr, uint16_t mot4_thr);
float get_bat_volt();
void led_set_brightness(uint8_t percent);
void led_breathe(uint8_t speed);
#endif