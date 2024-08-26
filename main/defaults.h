#ifndef DEFAULTS_H
#define DEFAULTS_H

#include "typedefs.h"

#define DFLT_MAX_PITCH_ANGLE         20.0f
#define DFLT_MAX_ROLL_ANGLE          20.0f
#define DFLT_MAX_PITCH_RATE          200.0f
#define DFLT_MAX_ROLL_RATE           200.0f
#define DFLT_MAX_YAW_RATE            150.0f
#define DFLT_PITCH_RATE_SCALE        8.0f
#define DFLT_ROLL_RATE_SCALE         8.0f
#define DFLT_YAW_RATE_SCALE          1.0f
#define DFLT_ALT_VEL_SCALE           0.4f
#define DFLT_MAX_VERTICAL_VELOCITY   1.0f
#define DFLT_MAX_HORIZONTAL_VELOCITY 0.3f
#define DFLT_TAKEOFF_ALTITUDE        1.0f
#define DFLT_VOLTAGE_SENS_GAIN       3.41f
#define DFLT_MAG_DECLINATION_DEG     0.0f
#define DFLT_HOVER_THROTTLE          460.0f
#define DFLT_PITCH_P                 2.0f
#define DFLT_PITCH_I                 1.0f
#define DFLT_PITCH_D                 13.0f
#define DFLT_ROLL_P                  2.0f
#define DFLT_ROLL_I                  1.0f
#define DFLT_ROLL_D                  13.0f
#define DFLT_YAW_P                   1.0f
#define DFLT_YAW_I                   0.0f
#define DFLT_POS_P                   30.0f
#define DFLT_POS_I                   2.0f
#define DFLT_ALT_P                   100.0f
#define DFLT_ALT_I                   2.0f
#define DFLT_ALT_D                   0.0f
#define DFLT_AHRS_FILTER_BETA        0.3f
#define DFLT_AHRS_FILTER_ZETA        0.003f



void load_default_config(config_t *cfg);
void load_default_config(config_t *cfg)
{
    cfg->max_pitch_angle = DFLT_MAX_PITCH_ANGLE;
    cfg->max_roll_angle = DFLT_MAX_ROLL_ANGLE;
    cfg->max_pitch_rate = DFLT_MAX_PITCH_RATE;
    cfg->max_roll_rate = DFLT_MAX_ROLL_RATE;
    cfg->max_yaw_rate = DFLT_MAX_YAW_RATE;
    cfg->max_vertical_velocity = DFLT_MAX_VERTICAL_VELOCITY;
    cfg->max_horizontal_velocity = DFLT_MAX_HORIZONTAL_VELOCITY;
    cfg->pitch_rate_scale = DFLT_PITCH_RATE_SCALE;
    cfg->roll_rate_scale = DFLT_ROLL_RATE_SCALE;
    cfg->yaw_rate_scale = DFLT_YAW_RATE_SCALE;
    cfg->takeoff_altitude = DFLT_TAKEOFF_ALTITUDE;
    cfg->voltage_sens_gain = DFLT_VOLTAGE_SENS_GAIN;
    cfg->mag_declination_deg = DFLT_MAG_DECLINATION_DEG;
    cfg->hover_throttle = DFLT_HOVER_THROTTLE;
    cfg->pitch_p = DFLT_PITCH_P;
    cfg->pitch_i = DFLT_PITCH_I;
    cfg->pitch_d = DFLT_PITCH_D;
    cfg->roll_p = DFLT_ROLL_P;
    cfg->roll_i = DFLT_ROLL_I;
    cfg->roll_d = DFLT_ROLL_D;
    cfg->yaw_p = DFLT_YAW_P;
    cfg->yaw_i = DFLT_YAW_I;
    cfg->pos_p = DFLT_POS_P;
    cfg->pos_i = DFLT_POS_I;
    cfg->alt_p = DFLT_ALT_P;
    cfg->alt_i = DFLT_ALT_I;
    cfg->alt_d = DFLT_ALT_D;
    cfg->ahrs_filter_beta = DFLT_AHRS_FILTER_BETA;
    cfg->ahrs_filter_zeta = DFLT_AHRS_FILTER_ZETA;
    cfg->alt_vel_scale = DFLT_ALT_VEL_SCALE;

    printf("Default Config Loaded...\n");
}


#endif