#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdio.h>
#include "bmp390.h"
#include "comminication.h"

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f
#define RANGE_BARO_TRANS_START_ALT 4.0f
#define RANGE_BARO_TRANS_END_ALT 6.0f

typedef struct 
{
    float pitch_deg;
    float roll_deg;
    float heading_deg;
    float pitch_dps;
    float roll_dps;
    float yaw_dps;
    float altitude_m;
    float vel_forward_ms;
    float vel_right_ms;
    float vel_up_ms;
    float acc_forward_ms2;
    float acc_right_ms2;
    float acc_up_ms2;
} states_t;


void ahrs_init(config_t *cfg, states_t *sta, imu_t *icm, magnetometer_t *hmc, bmp390_t *baro);
void ahrs_predict();
void ahrs_correct();
void get_earth_frame_accel();
void predict_altitude_velocity();
void correct_altitude_velocity();
void calculate_altitude_velocity();

#endif /*STATE_ESTIMATOR_H*/