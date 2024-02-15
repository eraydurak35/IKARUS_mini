#ifndef STATE_ESTIMATOR_H
#define STATE_ESTIMATOR_H

#include <stdio.h>
#include "icm42688p.h"
#include "hmc5883l.h"
#include "bmp390.h"
#include "comminication.h"

#define RAD_TO_DEG 57.29577951f
#define DEG_TO_RAD 0.01745329f

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


void ahrs_init(config_t *cfg, states_t *sta, icm42688p_t *icm, hmc5883l_t *hmc, bmp390_t *baro);
void ahrs_predict();
void ahrs_correct();
void get_earth_frame_accel();
void predict_altitude_velocity();
void correct_altitude_velocity();


/* void ahrs_init(icm42688p_t *imu, states_t *state, config_t *cfg);
void ahrs_predict(icm42688p_t *imu, states_t *state);
void ahrs_correct(icm42688p_t *imu, states_t *state); */

#endif /*STATE_ESTIMATOR_H*/