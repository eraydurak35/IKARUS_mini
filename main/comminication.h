#ifndef COMM_H
#define COMM_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "nvs_flash.h"

#define CHANNEL 6
#define DATARATE WIFI_PHY_RATE_54M//WIFI_PHY_RATE_24M

#define TELEM_HEADER 0xFF
#define CONF_HEADER 0xFE
#define WP_HEADER 0xFD
#define MTR_TEST_HEADER 0xFC


//esp_err_t esp_wifi_set_max_tx_power(int8_t power);
/*   * @attention This API gets maximum WiFi transmiting power. Values got
  *            from power are mapped to transmit power levels as follows:
  *            - 78: 19.5dBm
  *            - 76: 19dBm
  *            - 74: 18.5dBm
  *            - 68: 17dBm
  *            - 60: 15dBm
  *            - 52: 13dBm
  *            - 44: 11dBm
  *            - 34: 8.5dBm
  *            - 28: 7dBm
  *            - 20: 5dBm
  *            - 8:  2dBm
  *            - -4: -1dBm */
typedef struct
{
    float battery_voltage;
    float pitch;
    float roll;
    float heading;

    int16_t altitude;
    int16_t altitude_calibrated;
    int16_t tof_distance_1;
    int16_t tof_distance_2;
    int16_t velocity_x_ms;
    int16_t velocity_y_ms;
    int16_t velocity_z_ms;
    int16_t flow_x_velocity;
    int16_t flow_y_velocity;

    uint8_t flow_quality;
    uint8_t flight_mode;
    uint8_t arm_status;

    float target_pitch;
    float target_roll;
    float target_heading;
    float target_pitch_dps;
    float target_roll_dps;
    float target_yaw_dps;
    float target_altitude;
    float target_velocity_x_ms;
    float target_velocity_y_ms;
    float target_velocity_z_ms;

    int16_t barometer_pressure; // uint16_t olabilir
    uint16_t barometer_temperature; // int16_t olmalı
    uint16_t imu_temperature;
    int16_t gyro_x_dps;
    int16_t gyro_y_dps;
    int16_t gyro_z_dps;
    int16_t acc_x_ms2;
    int16_t acc_y_ms2;
    int16_t acc_z_ms2;
    int16_t mag_x_mgauss;
    int16_t mag_y_mgauss;
    int16_t mag_z_mgauss;

    uint8_t gps_fix;
    uint8_t gps_satCount;
    
    int32_t gps_latitude;
    int32_t gps_longitude;
    int32_t gps_altitude_m;
    int32_t gps_northVel_ms;
    int32_t gps_eastVel_ms;
    int32_t gps_downVel_ms;
    int32_t gps_headingOfMotion;

    uint16_t gps_hdop;
    uint16_t gps_vdop;

    int32_t gps_latitude_origin;
    int32_t gps_longitude_origin;
    int32_t gps_altitude_origin;
    int32_t target_latitude;
    int32_t target_longitude;
    float distance_m_2d;
    float distance_m_3d;
    float velocity_ms_2d;
} __attribute__((packed)) telemetry_t;


typedef struct
{
    int analog_LX;
    int analog_LY;
    int analog_RX;
    int analog_RY;
    int analog_LB;
    int analog_RB;
    int left_trigger;
    int right_trigger;
    int left_shoulder;
    int right_shoulder;
    int button_A;
    int button_B;
    int button_X;
    int button_Y;
} gamepad_t;

typedef struct
{
    float pitch_p;
    float pitch_i;
    float pitch_d;

    float roll_p;
    float roll_i;
    float roll_d;

    float yaw_p;
    float yaw_i;

    float ff_gain;

    float position_p;
    float position_i;

    float altitude_p;
    float altitude_i;
    float altitude_d;

    float max_pitch_angle;
    float max_roll_angle;
    float max_pitch_rate;
    float max_roll_rate;
    float max_yaw_rate;

    float pitch_rate_scale;
    float roll_rate_scale;
    float yaw_rate_scale;
    float max_vertical_velocity;
    float max_horizontal_velocity;

    float v_sens_gain;
    float v_drop_compensation_gain;
    float takeoff_altitude;
    float hover_throttle;
    float notch_1_freq;
    float notch_1_bandwidth;
    float notch_2_freq;
    float notch_2_bandwidth;
    float ahrs_filter_beta;
    float ahrs_filter_zeta;
    float alt_filter_beta;
    float mag_declination_deg;
    float velz_filter_beta;
    float velz_filter_zeta;
    float velxy_filter_beta;
    //
    float alt_to_vel_gain;
    float wp_threshold_cm;
    float wp_heading_correct_gain;
    float wp_dist_to_vel_gain;
} config_t;


typedef struct
{
    int32_t latitude[25];
    int32_t longitude[25];
    uint8_t altitude[25];
    int8_t counter;
    uint8_t is_reached;
} waypoint_t;


void comminication_init(gamepad_t *gmpd, config_t *cfg, float *mg_cal, uint8_t *flg, uint8_t *mtr_tst);
void comm_send_telem(telemetry_t *telem);
void comm_send_conf(config_t *conf);
void comm_send_wp();
void comm_send_motor_test_result(float *result);



#endif