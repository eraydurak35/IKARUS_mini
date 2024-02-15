#ifndef GPIO_H
#define GPIO_H
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define BUZZER_PIN GPIO_NUM_15
#define GREEN_PIN GPIO_NUM_27
#define RED_PIN GPIO_NUM_4
#define BLUE_PIN GPIO_NUM_14

#define VSENS_CHANNEL ADC_CHANNEL_6 //GPIO_NUM_34

#define MOT1_PIN GPIO_NUM_26
#define MOT2_PIN GPIO_NUM_25
#define MOT3_PIN GPIO_NUM_16
#define MOT4_PIN GPIO_NUM_17


void gpio_configure();
void set_throttle(uint16_t mot1_thr, uint16_t mot2_thr, uint16_t mot3_thr, uint16_t mot4_thr);
float get_bat_volt();
#endif