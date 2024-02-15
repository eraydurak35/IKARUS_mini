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
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "driver/spi_master.h"
// ||############################||
// ||      CUSTOM LIBRARIES      ||
// ||############################||
#include "comminication.h"
#include "i2c.h"
#include "filters.h"
#include "bmp390.h"
#include "uart.h"
#include "hmc5883l.h"
#include "spi.h"
#include "icm42688p.h"
#include "pmw3901.h"
#include "gpio.h"
#include "state_estimator.h"
#include "control_algorithm.h"
#include "nv_storage.h"

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
static fir_filter_t fir[6];
static uart_data_t uart_1_recv;
static TaskHandle_t task1_handler;
static TaskHandle_t task2_handler;
static TaskHandle_t task3_handler;

void IRAM_ATTR timer1_callback(void *arg)
{
    // xTaskResumeFromISR(task1_handler); // Task 1 runs 1000Hz
    xTaskNotifyFromISR(task1_handler, 1, eIncrement, false);
}

void task_1(void *pvParameters)
{
    static uint8_t counter1 = 0;
    static uint8_t counter2 = 0;
    static uint8_t counter3 = 0;
    static uint32_t receivedValue = 0;

    comminication_init(&gamepad, &config);
    read_config(&config);
    comm_send_conf(&config);
    gpio_configure();
    telemetry.battery_voltage = get_bat_volt() * config.v_sens_gain;
    fir_filter_init(fir);
    spi_master_init(MISO_PIN, MOSI_PIN, SCL_PIN);
    icm42688p_setup();
    bmp390_setup_spi();

    // fill the filter buffer before ahrs init
    for (uint8_t i = 0; i <= 100; i++)
    {
        icm42688p_read(&imu);
        apply_fir_filter_to_imu(&imu, fir);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }

    baro_set_ground_pressure(&barometer);

    ahrs_init(&config, &states, &imu, &mag, &barometer);
    control_init(&gamepad, &telemetry, &flight, &target, &states, &config);

    while (1)
    {
        if (xTaskNotifyWait(0, ULONG_MAX, &receivedValue, 1 / portTICK_PERIOD_MS) == pdTRUE)
        {
            // vTaskSuspend(NULL);
            icm42688p_read(&imu);
            apply_fir_filter_to_imu(&imu, fir);
            ahrs_predict();
            //printf("1\n");

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
                telemetry.gyro_x_dps = states.pitch_dps;
                telemetry.gyro_y_dps = states.roll_dps;
                telemetry.gyro_z_dps = states.yaw_dps;
                telemetry.acc_x_ms2 = imu.accel_ms2[X];
                telemetry.acc_y_ms2 = imu.accel_ms2[Y];
                telemetry.acc_z_ms2 = imu.accel_ms2[Z];

                telemetry.mag_x_mgauss = mag.axis[X];
                telemetry.mag_y_mgauss = mag.axis[Y];
                telemetry.mag_z_mgauss = mag.axis[Z];

                telemetry.target_heading = target.heading_deg;
                telemetry.target_yaw_dps = target.yaw_dps;
                telemetry.target_pitch = target.pitch_deg;
                telemetry.target_pitch_dps = target.pitch_dps;
                telemetry.target_roll = target.roll_deg;
                telemetry.target_roll_dps = target.roll_dps;

                telemetry.barometer_pressure = barometer.press;
                telemetry.barometer_temperature = barometer.temp;
                telemetry.imu_temperature = imu.temp_mC / 100.0f;
                telemetry.arm_status = flight.arm_status;
                telemetry.altitude = barometer.altitude_m;
                telemetry.altitude_calibrated = states.altitude_m;
                telemetry.velocity_z_ms = states.vel_up_ms;
                telemetry.target_altitude = target.altitude;
                telemetry.target_velocity_z_ms = target.velocity_z_ms;

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
    hmc5883l_setup();
    while (1)
    {
        hmc5883l_read(&mag);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

void app_main()
{

    xTaskCreatePinnedToCore(&task_3, "task3", 1024 * 4, NULL, 1, &task3_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_2, "task2", 1024 * 4, NULL, 1, &task2_handler, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(&task_1, "task1", 1024 * 4, NULL, 1, &task1_handler, tskNO_AFFINITY);

    esp_timer_handle_t timer1;
    const esp_timer_create_args_t timer1_args =
        {
            .callback = &timer1_callback,
            .arg = NULL,
            .name = "timer1"};
    esp_timer_create(&timer1_args, &timer1);
    esp_timer_start_periodic(timer1, 1000);

    vTaskDelay(500 / portTICK_PERIOD_MS);
    puts("                                                       -:");
    puts("                                                       @%");
    puts("                                                      %@@.");
    puts("                                                     -@@@.");
    puts("                                                    .@@@@. ..");
    puts("                                                   .@@@@@  *@.");
    puts("                    ..-+#@@@@@@@@%*=:.             %@@@@= .@@=");
    puts("                .+@@@@@@@@@@@@@@@@@@@@@@%.        #@@@@@ .@@@+");
    puts("             .@@@@@@#:..          ...+@@@. .:    @@@@@@  @@@@#");
    puts("          .=@@@@%.   ..::-=++=:..         .@@: .@@@@@@..@@@@@*");
    puts("        .#@@@%.  .+%%%%%%%%%%#=.           .  .@@@@@@= @@@*@@=");
    puts("       -@@@+.  =%%=:.          -@. .+-       -@@@@@@- @@%%@@@:");
    puts("     .@@@@. .=-   ..........-@@@# .@@@-     @@@@@@@ .@%%%:@@@  .%-");
    puts("    .@@@=    .+@@@@@@@@@@@@@@@@@    ..    :@@@@@@#.-%%%%:@@= .%@@.");
    puts("   .@@@.   .@@@@#-.  .::::.@@@*         .@@@@@@@::%%%%#.-. .@@@@+");
    puts("   @@@:   @@@# .@@.     -@@@@.  :#.    *@@@@@@:.#%%%%-   -@@@@@%");
    puts("  %@@=  +@@@..@@+...    .@@@@@@@@.  .#@@@@@@+.*%%%%*  :@@@%%@@%");
    puts(" :@@@ :@@@=   ...:=*+.     @@@@.  .@@@@@@@-.#%%%%+..@@@@%-=@@*");
    puts(" #@@: @@@.:..      .:*:    .@@@.-@@@@@@@::%%%%*..@@@%%=..@@%.  :*.");
    puts(".@@@ .@@+.@@@@@@@@:  .-     .@@@@@@@@: -%%%#..#%%%%+   .-  .%@@@.");
    puts(".@@@  @@@@@@.   *@@      .-@@@@@@@- .*%%*. .*%%%=.    .-%@@@@@=");
    puts(".@@@   -@@@@    *@@    .@@@@@@@:  -%#-.  :%%+.     =@@@@@@@@=");
    puts(".::.           #@@-  *@@@@@*.   ...   .::.     +@@@@@@@@@:. .#.");
    puts("  ...  .@@=  :@@@..*@@@@@.               .-@@@@@@@@@%..:+@@@@.");
    puts(" -@@@   =@. *@@# =@@@@- ..          .=@@@@@@@@%=.:+#%%%@@@@-");
    puts("  #@@:     =@@= %@@@:  :+.      .*@@@@@#=:.-*%%%%%%=*@@@@.");
    puts("   -@@.   .@@* @@@+  .%*     .#%:...:+%%%%%%%%- .-@@@#");
    puts(" .. .@*   @@@ @@@+  .%#    .:-+%%%%%%%%%%+.   .*%-  .-=*@@@.");
    puts(" .@+  .   @@*-@@=  .#%.           .%%-.  .:+%@@@@@@@@@@@@-");
    puts("  .@@.    *@:@@%   .%*          -@@:.#@@@@@@@@@@@@-@@@@.");
    puts("    @@@.  .@:@@  =.-%+        :@@@@@@@@@@@@@@. .+@@@-");
    puts("     *@@@.   #@ .@+.%*      :@@@@@@@@@@@+    .+%=.");
    puts("      .@@@%  .- :@@.%%.    @@@@#-.  -@@@@@@:");
    puts("        .@@@@.  .@@%.%%. :=         :+#%%#*-.   ...=.");
    puts("          .@@@@-.:@@@.@%:       .*%*+=====+*%@@@@#.");
    puts("            .=#@@- @@@.%@@:       .=@@@@@@@@@*-.");
    puts("            .-. .#@+.@@@@:");
    puts("                    .  #@@@@=.");
    puts("                         .*@@@@.");
    puts("                             .:-.");
    puts("// IKARUS*mini");
}
