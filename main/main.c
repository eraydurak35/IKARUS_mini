/*
8888888 888    d8P         d8888 8888888b.  888     888  .d8888b.                     d8b          d8b
  888   888   d8P         d88888 888   Y88b 888     888 d88P  Y88b                    Y8P          Y8P
  888   888  d8P         d88P888 888    888 888     888 Y88b.
  888   888d88K         d88P 888 888   d88P 888     888  "Y888b.        88888b.d88b.  888 88888b.  888
  888   8888888b       d88P  888 8888888P"  888     888     "Y88b.      888 "888 "88b 888 888 "88b 888
  888   888  Y88b     d88P   888 888 T88b   888     888       "888      888  888  888 888 888  888 888
  888   888   Y88b   d8888888888 888  T88b  Y88b. .d88P Y88b  d88P      888  888  888 888 888  888 888
8888888 888    Y88b d88P     888 888   T88b  "Y88888P"   "Y8888P"       888  888  888 888 888  888 888
*/
// colossal

//FOR ESP32-S3-MINI

// ||############################||
// ||      ESP IDF LIBRARIES     ||
// ||############################||
#include "freertos/FreeRTOS.h"
#include "driver/spi_master.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include <stdio.h>
#include "math.h"
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "command_line_interface.h"
#include "control_algorithm.h"
#include "state_estimator.h"
#include "calibration.h"
#include "nv_storage.h"
#include "motor_test.h"
#include "icm42688p.h"
#include "defaults.h"
#include "web_comm.h"
#include "qmc5883l.h"
#include "typedefs.h"
#include "filters.h"
#include "bmp390.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"

static flight_t flight;
static target_t target;
static config_t config;
static gamepad_t gamepad;
static states_t states;
static imu_t imu;
static bmp390_t barometer;
static magnetometer_t mag;
static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static TaskHandle_t task4_handler;
static calibration_t mag_calibration_data;
static calibration_t accel_calibration_data;
static telemetry_small_t telem_small;
static biquad_lpf_t lpf[6];

void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}
// GPIO 0'a bağlı butona hem basıldığında hem çekildiğinde bu interrupt tetiklenir
void IRAM_ATTR button_ISR(void *arg)
{
    // Kalibrasyon görevine butonun durum bilgisini gönder
    xTaskNotifyFromISR(task4_handler, gpio_get_level(BUTTON_PIN), eSetValueWithOverwrite, false);
}


void task_1(void *pvParameters)
{
    static uint8_t counter1 = 0;
    static uint8_t counter2 = 0;
    static uint8_t counter3 = 0;
    static uint32_t receivedValue = 0;

    web_comm_init(&gamepad, &telem_small);
    biquad_lpf_array_init(lpf);
    bmp390_setup_spi();

    // fill the filter buffer before ahrs init
    for (uint8_t i = 0; i <= 100; i++)
    {
        icm42688p_read(&imu);
        apply_biquad_lpf_to_imu(&imu, lpf);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }

    baro_set_ground_pressure(&barometer);

    ahrs_init(&config, &states, &imu, &mag, &barometer);
    control_init(&gamepad, &telem_small, &flight, &target, &states, &config);
    telem_small.battery_voltage = get_bat_volt() * config.voltage_gain;

    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {

            icm42688p_read(&imu);
            
            apply_biquad_lpf_to_imu(&imu, lpf);
            /* printf("%.2f,%.2f,%.2f\n", imu.gyro_dps[X],imu.gyro_dps[Y],imu.gyro_dps[Z]); */
            ahrs_predict();
            //printf("%.2f,%.2f,%.2f\n", states.pitch_deg,states.roll_deg,states.heading_deg);

            counter1++;
            counter2++;
            counter3++;
            if (counter2 >= 2) // 500Hz
            {
                counter2 = 0;
                get_earth_frame_accel();
                ahrs_correct();
                predict_altitude_velocity();
                flight_control();
                    
            }
            if (counter1 >= 20) // 50Hz
            {
                counter1 = 0;
                bmp390_read_spi(&barometer);
                baro_get_altitude_velocity(&barometer);
                correct_altitude_velocity();
                check_flight_mode();
            }
            if (counter3 >= 100) // 10Hz
            {
                counter3 = 0;
                telem_small.battery_voltage = (get_bat_volt() * config.voltage_gain) * 0.01f + telem_small.battery_voltage * 0.99f;
                telem_small.pitch = states.pitch_deg;
                telem_small.roll = states.roll_deg;
                telem_small.heading = states.heading_deg;
                telem_small.altitude = states.altitude_m;

                wifi_sta_list_t sta_list;
                esp_err_t err = esp_wifi_ap_get_sta_list(&sta_list);
                if (err == ESP_OK)
                {
                    telem_small.rssi = sta_list.sta[0].rssi;
                }

                if (telem_small.rssi == 0) led_breathe(5);
                else led_set_brightness(70);
            }
        }
    }
}


void task_2(void *pvParameters)
{
    // Todo: sensörün who am i değerini oku cevap gelirse devam et
    i2c_master_init(I2C_NUM_0, SDA1, SCL1, 400000, GPIO_PULLUP_DISABLE);
    qmc5883l_setup(&mag_calibration_data);

    while (1)
    {
        qmc5883l_read(&mag, 0);
        //printf("%.2f,%.2f,%.2f\n", mag.axis[X], mag.axis[Y], mag.axis[Z]);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void task_3(void *pvParameters)
{
    gpio_configure();
    // Attach the interrupt handler to the GPIO
    gpio_isr_handler_add(BUTTON_PIN, button_ISR, (void*) BUTTON_PIN);
    led_set_brightness(0);
    spi_master_init(MISO_PIN, MOSI_PIN, CLK_PIN);
    icm42688p_setup(&accel_calibration_data);
    uint8_t blink_counter = 0;

    while (1)
    {
        icm42688p_read(&imu);
        blink_counter++;
        if (blink_counter == 249) blink_counter = 0;
        else if (blink_counter == 1) led_set_brightness(100);
        else if (blink_counter == 125) led_set_brightness(0);

        if (gyro_calibration(&imu) == 1)
        {
            led_set_brightness(70);
            xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);
            esp_timer_handle_t timer1;
            const esp_timer_create_args_t timer1_args =
            {
                .callback = &timer1_callback,
                .arg = NULL,
                .name = "timer1"
            };
            esp_timer_create(&timer1_args, &timer1);
            esp_timer_start_periodic(timer1, 900);
            puts("IKARUS");
            vTaskDelete(NULL);
            printf("THIS SHOULD NOT PRINT\n");
        }

        vTaskDelay(2);
    }
}

// İvme ölçer ve manyetik sensörün kalibrasyonunu gerçekleştiren görev
void task_4(void *pvParameters)
{
    static int64_t button_push_time_difference;
    static int64_t button_push_current_time;
    static uint32_t button_level;
    static uint8_t is_calibration_done = 0;

    while (1)
    {
        // Butona basıldığında veya çekildiğinde bu işlev çalışır. Bu durumlar dışında çalışmaz bekler.
        if (xTaskNotifyWait(0, ULONG_MAX, &button_level, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            // Eğer buton bırakıldıysa button level 1'dir. Basıldı ise 0'dır
            // Sadece buton bırakıldığında süre ölçümü yap
            if (button_level == 1)
            {
                // Butona basılması ile bırakılması arasındaki süreyi ölç
                button_push_time_difference = (esp_timer_get_time() - button_push_current_time);

                // Bu süre (button_push_time_difference) 1 saniyeden kısa ise ivme ölçer kalibrasyonu seçilmiştir (1000000 us = 1 sn)
                if (button_push_time_difference < 1000000)
                {
                    // 1. ve 2. görevleri silebiliriz. Çalışmalarına gerek yok.
                    vTaskDelete(task1_handler);
                    vTaskDelete(task2_handler);
                    // Eski kalibrasyon verilerini sıfırla ki yeni kalibrasyon yapabilelim.
                    reset_calibration_data(&accel_calibration_data);
                    // Kalibrasyon fonksiyonu 1 döndürene kadar döngü devam etsin
                    while (!is_calibration_done)
                    {
                        // IMU'dan yeni veri oku
                        icm42688p_read(&imu);
                        // Yeni veriyi kalibrasyon fonksiyonuna gönder
                        // Kalibrasyon süresince 0, bittiğinde 1 döndürür
                        is_calibration_done = accelerometer_calibration(&imu);
                        // 5ms bekle
                        vTaskDelay(5);
                    }
                    // Bu noktaya ulaştığında kalibrasyon tamamlanmış demektir.
                    // Soft restart at
                    esp_restart();
                }
                // Bu süre (button_push_time_difference) 1 saniyeden uzun ise manyetik sensör kalibrasyonu seçilmiştir
                else
                {
                    // 1. ve 2. görevleri silebiliriz. Çalışmalarına gerek yok.
                    vTaskDelete(task1_handler);
                    vTaskDelete(task2_handler);
                    // Eski kalibrasyon verilerini sıfırla ki yeni kalibrasyon yapabilelim.
                    reset_calibration_data(&mag_calibration_data);
                    // Kalibrasyon fonksiyonu 1 döndürene kadar döngü devam etsin
                    while (!is_calibration_done)
                    {
                        // Manyetik sensörden yeni veri oku
                        qmc5883l_read(&mag, 0);
                        // Yeni veriyi kalibrasyon fonksiyonuna gönder
                        // Kalibrasyon süresince 0, bittiğinde 1 döndürür
                        is_calibration_done = magnetometer_calibration(&mag);
                        // 50ms bekle
                        vTaskDelay(50);
                    }
                    // Bu noktaya ulaştığında kalibrasyon tamamlanmış demektir.
                    // Soft restart at
                    esp_restart();
                }
            }
            // Butona her basılıp çekildiğinde o anki zamanı kaydet
            // Bu sayede basıp çekme arasındaki zamanı ölçebilelim
            button_push_current_time = esp_timer_get_time();
        }
    }
}

void app_main()
{
    // Non Volatile Storage birimini başlatır.
    nvs_flash_init();
    cli_begin(&config, &accel_calibration_data, &mag_calibration_data, &imu);
    // Varsa önceden kaydedilmiş kalibrasyon verilerini ve konfigürasyonu oku
    // Yoksa default değerler ile başlat
    if (!storage_read(&mag_calibration_data, MAG_CALIB_DATA)) reset_calibration_data(&mag_calibration_data);
    if (!storage_read(&accel_calibration_data, ACCEL_CALIB_DATA)) reset_calibration_data(&accel_calibration_data);
    if (!storage_read(&config, CONFIG_DATA)) load_default_config(&config);

/*     printf("MAG offset X: %.3f  Y: %.3f  Z: %.3f\n", mag_calibration_data.offset[X], mag_calibration_data.offset[Y], mag_calibration_data.offset[Z]);
    printf("MAG scale X: %.3f  Y: %.3f  Z: %.3f\n", mag_calibration_data.scale[X], mag_calibration_data.scale[Y], mag_calibration_data.scale[Z]); 
    printf("ACCEL offset X: %.3f  Y: %.3f  Z: %.3f\n", accel_calibration_data.offset[X], accel_calibration_data.offset[Y], accel_calibration_data.offset[Z]);
    printf("ACCEL scale X: %.3f  Y: %.3f  Z: %.3f\n", accel_calibration_data.scale[X], accel_calibration_data.scale[Y], accel_calibration_data.scale[Z]); */

    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 4, NULL, 1, &task3_handler, tskNO_AFFINITY);
    // İvme ölçer ve manyetik sensör kalibrasyon görevi. Öncelik değeri (Idle = 0) olarak ayarlı
    xTaskCreatePinnedToCore(&task_4, "task4", 1024 * 4, NULL, 0, &task4_handler, tskNO_AFFINITY);
}
