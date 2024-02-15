#ifndef COMM_H
#define COMM_H

#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_private/wifi.h"
#include "nvs_flash.h"

#define CHANNEL 6
#define DATARATE WIFI_PHY_RATE_54M//WIFI_PHY_RATE_24M

#define TELEM_HEADER 0xFF
#define CONF_HEADER 0XFE


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
    float altitude;
    float altitude_calibrated;
    float tof_distance_1;
    float tof_distance_2;
    float velocity_x_ms;
    float velocity_y_ms;
    float velocity_z_ms;
    float flow_x_velocity;
    float flow_y_velocity;
    float flow_quality;
    float flight_mode;
    float arm_status;
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
    float barometer_pressure;
    float barometer_temperature;
    float imu_temperature;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
    float acc_x_ms2;
    float acc_y_ms2;
    float acc_z_ms2;
    float mag_x_mgauss;
    float mag_y_mgauss;
    float mag_z_mgauss;

    //==============//
    //     GPS      //
    //==============//
    float gps_fix;
    float gps_satCount;
    float gps_latitude;
    float gps_longitude;
    float gps_altitude_m;
    float gps_northVel_ms;
    float gps_eastVel_ms;
    float gps_downVel_ms;
    float gps_headingOfMotion;
    float gps_hdop;
    float gps_vdop;
    float gps_latitude_origin;
    float gps_longitude_origin;
    float gps_altitude_origin;
    float target_latitude;
    float target_longitude;
    float distance_m_2d;
    float distance_m_3d;
    float velocity_ms_2d;
} telemetry_t;


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

    float position_p;
    float position_i;
    float position_d;

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
} config_t;

void comminication_init(gamepad_t *gmpd, config_t *cfg);
void comm_send_telem(telemetry_t *telem);
void comm_send_conf(config_t *conf);



#endif