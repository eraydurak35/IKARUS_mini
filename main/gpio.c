#include "gpio.h"
#include "driver/ledc.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_timer.h"


adc_cali_handle_t adc1_cali_chan4_handle = NULL;
adc_oneshot_unit_handle_t adc1_handle;





// timer configurations
const ledc_timer_config_t ledc_timer1 = 
{
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 50000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
const ledc_timer_config_t ledc_timer2 = 
{
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 50000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
const ledc_timer_config_t ledc_timer3 = 
{
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 50000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
const ledc_timer_config_t ledc_timer4 = 
{
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 50000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_0
};
const ledc_timer_config_t ledc_timer5 = 
{
    .duty_resolution = LEDC_TIMER_10_BIT,
    .freq_hz = 1000,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = LEDC_TIMER_1
};




// channel configurations
const ledc_channel_config_t ledc_mot1 = 
{
    .gpio_num = MOT1_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

const ledc_channel_config_t ledc_mot2 = 
{
    .gpio_num = MOT2_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_1,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

const ledc_channel_config_t ledc_mot3 = 
{
    .gpio_num = MOT3_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_2,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

const ledc_channel_config_t ledc_mot4 = 
{
    .gpio_num = MOT4_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_3,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0
};

const ledc_channel_config_t ledc_led = 
{
    .gpio_num = LED_PIN,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_4,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_1,
    .duty = 0
};

static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten);

/// this function is not thread safe!!
void gpio_configure()
{
    ledc_timer_config(&ledc_timer1);
    ledc_timer_config(&ledc_timer2);
    ledc_timer_config(&ledc_timer3);
    ledc_timer_config(&ledc_timer4);
    ledc_timer_config(&ledc_timer5);

    ledc_channel_config(&ledc_mot1);
    ledc_channel_config(&ledc_mot2);
    ledc_channel_config(&ledc_mot3);
    ledc_channel_config(&ledc_mot4);
    ledc_channel_config(&ledc_led);

    set_throttle(0, 0, 0, 0);

    adc_oneshot_unit_init_cfg_t init_config1 = 
    {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = 
    {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11,
    };
    adc_oneshot_config_channel(adc1_handle, VSENS_CHANNEL, &config);
    adc_calibration_init(ADC_UNIT_1, VSENS_CHANNEL, ADC_ATTEN_DB_11);



    // GPIO 0 button interrupt config
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on both rising and falling edge
    io_conf.mode = GPIO_MODE_INPUT;        // Set as input mode
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_0); // Select GPIO 0
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    // Install GPIO ISR handler service
    gpio_install_isr_service(0);
}


static void adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
    adc_cali_curve_fitting_config_t cali_config = 
    {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    adc_cali_create_scheme_curve_fitting(&cali_config, &adc1_cali_chan4_handle);
}

float get_bat_volt()
{
    static int adc1_ch4_raw = 0;
    static int volt_raw = 0;
    adc_oneshot_read(adc1_handle, VSENS_CHANNEL, &adc1_ch4_raw);
    adc_cali_raw_to_voltage(adc1_cali_chan4_handle, adc1_ch4_raw, &volt_raw);
    return (float)(volt_raw / 1000.0f);
}


void set_throttle(uint16_t mot1_thr, uint16_t mot2_thr, uint16_t mot3_thr, uint16_t mot4_thr)
{
    ledc_set_duty(ledc_mot1.speed_mode, ledc_mot1.channel, mot1_thr);
    ledc_update_duty(ledc_mot1.speed_mode, ledc_mot1.channel);

    ledc_set_duty(ledc_mot2.speed_mode, ledc_mot2.channel, mot2_thr);
    ledc_update_duty(ledc_mot2.speed_mode, ledc_mot2.channel);

    ledc_set_duty(ledc_mot3.speed_mode, ledc_mot3.channel, mot3_thr);
    ledc_update_duty(ledc_mot3.speed_mode, ledc_mot3.channel);

    ledc_set_duty(ledc_mot4.speed_mode, ledc_mot4.channel, mot4_thr);
    ledc_update_duty(ledc_mot4.speed_mode, ledc_mot4.channel);
}

// LED'in parlaklığını yüzde cinsinden ayarlar
void led_set_brightness(uint8_t percent)
{
    ledc_set_duty(ledc_led.speed_mode, ledc_led.channel, percent * 10);
    ledc_update_duty(ledc_led.speed_mode, ledc_led.channel);
}
void led_breathe(uint8_t speed)
{
    static uint8_t value = 0;    // Başlangıç değeri
    static uint8_t increment = 1; // Artış miktarı
    value += increment * speed; // Değeri artır veya azalt

    // Değer 100 olduğunda veya 0 olduğunda artış yönünü değiştir
    if (value >= 100) 
    {
        increment = -1;
        value = 100;
    }
    else if (value <= 0) 
    {
        increment = 1;
        value = 0;
    }
    led_set_brightness(value);
}