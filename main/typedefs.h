#ifndef TYPEDEFS_H
#define TYPEDEFS_H

#include <stdio.h>

#define X 0
#define Y 1
#define Z 2

typedef struct
{
    float battery_voltage;
    int8_t rssi;
    float pitch;
    float roll;
    float heading;
    float altitude;
    uint8_t is_headless_on;
    uint8_t is_flip_on;
    uint8_t is_alt_hold_on;
} telemetry_small_t;

typedef struct
{
    uint8_t battery_voltage;
    int8_t rssi;
    int8_t pitch;
    int8_t roll;
    uint8_t heading;
    int8_t altitude;
    uint8_t is_headless_on;
    uint8_t is_flip_on;
    uint8_t is_alt_hold_on;
} __attribute__((packed)) telemetry_small_integer_t;

typedef struct
{
  float pitch_deg;
  float roll_deg;
  float heading_deg;
  float pitch_dps;
  float roll_dps;
  float yaw_dps;
  float altitude;
  float velocity_x_ms;
  float velocity_y_ms;
  float velocity_z_ms;
  float throttle;
} target_t;

typedef struct
{
    float axis[3];
} magnetometer_t;

typedef struct
{
    float gyro_dps[3];
    float accel_ms2[3];
    int16_t temp_mC;
    float gyro_bias_dps[3];
} imu_t;

typedef struct
{
  uint8_t arm_status;
  uint8_t alt_hold_status;
  uint8_t pos_hold_status;
  uint8_t takeoff_status;
} flight_t;

typedef struct
{
    float battery_voltage;
    float pitch;
    float roll;
    float heading;

    int16_t altitude;
    int16_t altitude_calibrated;
    int16_t tof_distance;
    int16_t throttle;
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
    uint8_t is_gnss_sanity_check_ok;
} __attribute__((packed)) telemetry_t;


typedef struct
{
    int8_t analog_LX;
    int8_t analog_LY;
    int8_t analog_RX;
    int8_t analog_RY;
    int8_t analog_LB;
    int8_t analog_RB;
    int8_t left_trigger;
    int8_t right_trigger;
    int8_t left_shoulder;
    int8_t right_shoulder;
    int8_t button_A;
    int8_t button_B;
    int8_t button_X;
    int8_t button_Y;
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
    float pos_p;
    float pos_i;
    float alt_p;
    float alt_i;
    float alt_d;
    float max_pitch_angle;
    float max_roll_angle;
    float max_pitch_rate;
    float max_roll_rate;
    float max_yaw_rate;
    float pitch_rate_scal;
    float roll_rate_scal;
    float yaw_rate_scale;
    float max_vert_vel;
    float max_horiz_vel;
    float voltage_gain;
    float takeoff_alt;
    float hover_throttle;
    float notch_1_freq;
    float notch_1_bndwdht;
    float notch_2_freq;
    float notch_2_bndwdht;
    float lpf_cutoff_hz;
    float ahrs_filt_beta;
    float ahrs_filt_zeta;
    float alt_filt_beta;
    float mag_declin_deg;
    float velz_filt_beta;
    float velz_filt_zeta;
    float velxy_filt_beta;
    float alt_vel_scale;
    float wp_threshold_cm;
    float wp_hdg_cor_gain;
    float wp_dis_vel_gain;
} config_t;

typedef struct {
    const char* name;
    float* value;
} key_value_t;

typedef struct
{
    int32_t latitude[25];
    int32_t longitude[25];
    uint8_t altitude[25];
    int8_t counter;
    uint8_t is_reached;
    uint8_t end_of_mission_behaviour;
} waypoint_t;

typedef struct
{
    float offset[3];
    float scale[3];
} calibration_t;

#endif