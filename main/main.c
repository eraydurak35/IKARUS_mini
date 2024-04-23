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
#include <stdio.h>
#include "math.h"
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "control_algorithm.h"
#include "state_estimator.h"
#include "comminication.h"
#include "nv_storage.h"
#include "motor_test.h"
#include "icm42688p.h"
#include "hmc5883l.h"
#include "filters.h"
#include "pmw3901.h"
#include "bmp390.h"
#include "uart.h"
#include "gpio.h"
#include "spi.h"
#include "i2c.h"
#include "buzzer.h"

static flight_t flight;
static target_t target;
static telemetry_t telemetry;
static config_t config;
static gamepad_t gamepad;
static states_t states;
static icm42688p_t imu;
static bmp390_t barometer;
static pmw3901_t flow;
static hmc5883l_t mag;
static uart_data_t uart_1_recv;
static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;
static TaskHandle_t beep_task_handler;
static uint8_t is_new_gsa_data_recv_flag = 0;
static float mag_calib_data[12] = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
static uint8_t motor_test_number = 0;

static biquad_lpf_t lpf[6];

void IRAM_ATTR timer1_callback(void *arg)
{
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}

void task_1(void *pvParameters)
{
    static uint8_t counter1 = 0;
    static uint8_t counter2 = 0;
    static uint8_t counter3 = 0;
    static uint32_t receivedValue = 0;

    comminication_init(&gamepad, &config, mag_calib_data, &is_new_gsa_data_recv_flag, &motor_test_number);
    read_config(&config);
    read_mag_cal(mag_calib_data);
    gpio_configure();
    biquad_lpf_array_init(lpf);
    spi_master_init(MISO_PIN, MOSI_PIN, SCL_PIN);
    icm42688p_setup();
    bmp390_setup_spi();
    telemetry.battery_voltage = get_bat_volt() * config.v_sens_gain;

    telemetry.gps_latitude = 391108961;
    telemetry.gps_longitude = 271877099;

    // fill the filter buffer before ahrs init
    for (uint8_t i = 0; i <= 100; i++)
    {
        icm42688p_read(&imu);
        apply_biquad_lpf_to_imu(&imu, lpf);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }

    baro_set_ground_pressure(&barometer);

    ahrs_init(&config, &states, &imu, &mag, &barometer);
    control_init(&gamepad, &telemetry, &flight, &target, &states, &config);

    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            icm42688p_read(&imu);

            if (motor_test_number > 0)
            {
                uint8_t ret = motor_test(&imu, motor_test_number);
                if (ret == 0)
                {
                    motor_test_number = 0;
                    comm_send_motor_test_result(get_motor_test_results());
                }
            }

            apply_biquad_lpf_to_imu(&imu, lpf);
            ahrs_predict();

            if (is_new_gsa_data_recv_flag > 0)
            {
                xTaskNotify(beep_task_handler, is_new_gsa_data_recv_flag, eSetValueWithOverwrite);
                if (is_new_gsa_data_recv_flag == 3)
                    comm_send_conf(&config);
                else if (is_new_gsa_data_recv_flag == 4)
                    comm_send_wp();
                is_new_gsa_data_recv_flag = 0;
            }

            counter1++;
            counter2++;
            counter3++;
            if (counter2 >= 2) // 500Hz
            {
                counter2 = 0;
                get_earth_frame_accel();
                ahrs_correct();
                calculate_altitude_velocity();
                //predict_altitude_velocity();
                if (motor_test_number == 0)
                {
                    flight_control();
                }
                    
            }
            if (counter1 >= 20) // 50Hz
            {
                counter1 = 0;
                bmp390_read_spi(&barometer);
                baro_get_altitude_velocity(&barometer);
                //correct_altitude_velocity();
                // printf("%.2f,%.2f\n", states.altitude_m, barometer.altitude_m);
                // printf("%.2f,%.2f\n", states.vel_up_ms, barometer.velocity_ms);
                check_flight_mode();
            }
            if (counter3 >= 100) // 10Hz
            {
                counter3 = 0;
                telemetry.battery_voltage = (get_bat_volt() * config.v_sens_gain) * 0.005f + telemetry.battery_voltage * 0.995f;
                telemetry.pitch = states.pitch_deg;
                telemetry.roll = states.roll_deg;
                telemetry.heading = states.heading_deg;
                telemetry.gyro_x_dps = states.pitch_dps * 100.0f;
                telemetry.gyro_y_dps = states.roll_dps * 100.0f;
                telemetry.gyro_z_dps = states.yaw_dps * 100.0f;
                telemetry.acc_x_ms2 = imu.accel_ms2[X] * 400.0f;
                telemetry.acc_y_ms2 = imu.accel_ms2[Y] * 400.0f;
                telemetry.acc_z_ms2 = imu.accel_ms2[Z] * 400.0f;

                telemetry.mag_x_mgauss = mag.axis[X] * 10.0f;
                telemetry.mag_y_mgauss = mag.axis[Y] * 10.0f;
                telemetry.mag_z_mgauss = mag.axis[Z] * 10.0f;

                telemetry.target_heading = target.heading_deg;
                telemetry.target_yaw_dps = target.yaw_dps;
                telemetry.target_pitch = target.pitch_deg;
                telemetry.target_pitch_dps = target.pitch_dps;
                telemetry.target_roll = target.roll_deg;
                telemetry.target_roll_dps = target.roll_dps;

                telemetry.barometer_pressure = barometer.press * 10.0f;
                telemetry.barometer_temperature = barometer.temp * 100.0;
                telemetry.imu_temperature = imu.temp_mC;
                telemetry.arm_status = flight.arm_status;
                telemetry.altitude = barometer.altitude_m * 100.0f;
                telemetry.altitude_calibrated = states.altitude_m * 100.0f;
                telemetry.velocity_z_ms = states.vel_up_ms * 1000.0f;
                telemetry.target_altitude = target.altitude;
                telemetry.target_velocity_z_ms = target.velocity_z_ms;

                telemetry.gps_latitude += 1;
                telemetry.gps_longitude += 1;

                if (flight.alt_hold_status)
                {
                    if (flight.pos_hold_status)
                        telemetry.flight_mode = 3;
                    else
                        telemetry.flight_mode = 1;
                }
                else if (flight.pos_hold_status)
                    telemetry.flight_mode = 2;
                else
                    telemetry.flight_mode = 0;

                comm_send_telem(&telemetry);
            }
        }
    }
}

void task_2(void *pvParameters)
{
    uart_begin(UART_NUM_1, 19200, GPIO_NUM_12, GPIO_NUM_35, UART_PARITY_DISABLE);
    while (1)
    {
        // this blocks task for timeout period
        uart_read(UART_NUM_1, &uart_1_recv, 10);
        parse_pmw3901_data(&flow, &uart_1_recv);
    }
}

void task_3(void *pvParameters)
{
    i2c_master_init(I2C_NUM_0, SDA1, SCL1, 400000, GPIO_PULLUP_DISABLE);
    hmc5883l_setup(mag_calib_data);

    while (1)
    {
        hmc5883l_read(&mag);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void task_4(void *pvParameters)
{
    static uint32_t notification = 0;
    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &notification, portMAX_DELAY) == pdTRUE)
        {
            play_notification(notification);
        }
    }
}

void app_main()
{

    xTaskCreatePinnedToCore(&task_4, "task4", 1024 * 4, NULL, 0, &beep_task_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 4, NULL, 1, &task3_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);

    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args =
    {
        .callback = &timer1_callback,
        .arg = NULL,
        .name = "timer1"
    };
    esp_timer_create(&timer1_args, &timer1);
    esp_timer_start_periodic(timer1, 1000);
    puts("// IKARUS*mini");
}
