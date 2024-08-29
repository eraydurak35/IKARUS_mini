#include "defaults.h"
#include "typedefs.h"

void load_default_config(config_t *cfg)
{
    cfg->max_pitch_angle = DFLT_MAX_PITCH_ANGLE;
    cfg->max_roll_angle = DFLT_MAX_ROLL_ANGLE;
    cfg->max_pitch_rate = DFLT_MAX_PITCH_RATE;
    cfg->max_roll_rate = DFLT_MAX_ROLL_RATE;
    cfg->max_yaw_rate = DFLT_MAX_YAW_RATE;
    cfg->max_vert_vel = DFLT_MAX_VERTICAL_VELOCITY;
    cfg->max_horiz_vel = DFLT_MAX_HORIZONTAL_VELOCITY;
    cfg->pitch_rate_scal = DFLT_PITCH_RATE_SCALE;
    cfg->roll_rate_scal = DFLT_ROLL_RATE_SCALE;
    cfg->yaw_rate_scale = DFLT_YAW_RATE_SCALE;
    cfg->takeoff_alt = DFLT_TAKEOFF_ALTITUDE;
    cfg->voltage_gain = DFLT_VOLTAGE_SENS_GAIN;
    cfg->mag_declin_deg = DFLT_MAG_DECLINATION_DEG;
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
    cfg->ahrs_filt_beta = DFLT_AHRS_FILTER_BETA;
    cfg->ahrs_filt_zeta = DFLT_AHRS_FILTER_ZETA;
    cfg->alt_vel_scale = DFLT_ALT_VEL_SCALE;

    printf("Default Config Loaded...\n");
}