#include "state_estimator.h"
#include "icm42688p.h"
#include "hmc5883l.h"
#include "bmp390.h"
#include "math.h"
#include "comminication.h"
#include "quaternion.h"

// An Extended Complementary Filter (ECF) for Full-Body MARG Orientation Estimation
// DOI:10.1109/TMECH.2020.2992296

static quat_t q = {1.0f, 0.0f, 0.0f, 0.0f};
static quat_t q_dot = {0.0f, 0.0f, 0.0f, 0.0f};
static vector_t gyr_vec = {0.0f, 0.0f, 0.0f};
static vector_t acc_vec = {0.0f, 0.0f, 1.0f};
static vector_t mag_vec = {1.0f, 0.0f, 0.0f};
static vector_t err = {0.0f, 0.0f, 0.0f};
static vector_t local_vr_a = {0.0f, 0.0f, 0.0f};
//static vector_t local_vr_m = {0.0f, 0.0f, 0.0f};

static states_t *state_ptr = NULL;
static icm42688p_t *imu_ptr = NULL;
static hmc5883l_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static config_t *config_ptr = NULL;

static float acc_z_bias = 0;
static void get_attitude_heading();

void ahrs_init(config_t *cfg, states_t *sta, icm42688p_t *icm, hmc5883l_t *hmc, bmp390_t *baro)
{
    config_ptr = cfg;
    state_ptr = sta;
    imu_ptr = icm;
    mag_ptr = hmc;
    baro_ptr = baro;

    acc_vec.x = imu_ptr->accel_ms2[Y];
    acc_vec.y = imu_ptr->accel_ms2[X];
    acc_vec.z = imu_ptr->accel_ms2[Z];

    mag_vec.x = mag_ptr->axis[X];
    mag_vec.y = mag_ptr->axis[Y];
    mag_vec.z = mag_ptr->axis[Z];

    norm_vector(&acc_vec);
    norm_vector(&mag_vec);

    quat_t q_acc, q_mag;
    get_attitude_from_accel(&acc_vec, &q_acc);
    get_heading_from_mag(&mag_vec, &q_mag);
    q = get_quat_product(&q_acc, &q_mag);
}

void ahrs_predict()
{
    gyr_vec.x = imu_ptr->gyro_dps[Y] * DEG_TO_RAD;
    gyr_vec.y = imu_ptr->gyro_dps[X] * DEG_TO_RAD;
    gyr_vec.z = -imu_ptr->gyro_dps[Z] * DEG_TO_RAD;

    gyr_vec.x -= (config_ptr->ahrs_filter_beta * err.x);
    gyr_vec.y -= (config_ptr->ahrs_filter_beta * err.y);
    gyr_vec.z -= (config_ptr->ahrs_filter_beta * err.z);

    get_quat_deriv(&q, &gyr_vec, &q_dot);

    q.w += q_dot.w / 1000.0f;
    q.x += q_dot.x / 1000.0f;
    q.y += q_dot.y / 1000.0f;
    q.z += q_dot.z / 1000.0f;

    norm_quat(&q);
}

static void get_attitude_heading()
{
    static quat_t q_conj;
    static vector_t euler;

    q_conj = quat_conj(&q);
    quat_to_euler(&q_conj, &euler);

    state_ptr->pitch_deg = euler.x;
    state_ptr->roll_deg = euler.y;
    state_ptr->heading_deg = euler.z;

    state_ptr->pitch_dps = imu_ptr->gyro_dps[X];
    state_ptr->roll_dps = imu_ptr->gyro_dps[Y];
    state_ptr->yaw_dps = -imu_ptr->gyro_dps[Z];
}

void ahrs_correct()
{
    static vector_t err_acc = {0.0f, 0.0f, 0.0f};
    static vector_t err_mag = {0.0f, 0.0f, 0.0f};
    //static vector_t temp = {0.0f, 0.0f, 0.0f};

    acc_vec.x = imu_ptr->accel_ms2[Y];
    acc_vec.y = imu_ptr->accel_ms2[X];
    acc_vec.z = -imu_ptr->accel_ms2[Z];

    mag_vec.x = mag_ptr->axis[X];
    mag_vec.y = -mag_ptr->axis[Y];
    mag_vec.z = mag_ptr->axis[Z];

    norm_vector(&acc_vec);
    norm_vector(&mag_vec);

    local_vr_a.x = 2.0f * ((q.x * q.z) - (q.w * q.y));
    local_vr_a.y = 2.0f * ((q.w * q.x) + (q.y * q.z));
    local_vr_a.z = 2.0f * ((q.w * q.w) + (q.z * q.z)) - 1.0f;

    err_acc = cross_product(&acc_vec, &local_vr_a);

/*  local_vr_m.x = 2.0f * ((q.x * q.y) + (q.w * q.z));
    local_vr_m.y = 2.0f * ((q.w * q.w) + (q.y * q.y)) - 1.0f;
    local_vr_m.z = 2.0f * ((q.y * q.z) - (q.w * q.x));

    temp = cross_product(&acc_vec, &mag_vec);
    err_mag = cross_product(&temp, &local_vr_m);
*/

    err.x = err_acc.x + err_mag.x;
    err.y = err_acc.y + err_mag.y;
    err.z = err_acc.z + err_mag.z;

    imu_ptr->gyro_bias_dps[Y] += err.x * config_ptr->ahrs_filter_zeta;
    imu_ptr->gyro_bias_dps[X] += err.y * config_ptr->ahrs_filter_zeta;
    //imu_ptr->gyro_bias_dps[Z] -= err.z * config_ptr->ahrs_filter_zeta;

    get_attitude_heading();
}


void get_earth_frame_accel()
{
    static float rot_matrix[3][3] = {0};
    static float pitch_radians = 0.0f;
    static float roll_radians = 0.0f;
    static float cosx = 0.0f;
    static float sinx = 0.0f;
    static float cosy = 0.0f;
    static float siny = 0.0;

    pitch_radians = state_ptr->pitch_deg * DEG_TO_RAD;
    roll_radians = state_ptr->roll_deg * DEG_TO_RAD;

    cosx = cosf(roll_radians);
    sinx = sinf(roll_radians);
    cosy = cosf(pitch_radians);
    siny = sinf(pitch_radians);

    rot_matrix[0][0] = cosy;
    rot_matrix[0][1] = 0.0f;
    rot_matrix[0][2] = siny;
    rot_matrix[1][0] = sinx * siny;
    rot_matrix[1][1] = cosx;
    rot_matrix[1][2] = -sinx * cosy;
    rot_matrix[2][0] = -(cosx * siny);
    rot_matrix[2][1] = sinx;
    rot_matrix[2][2] = cosy * cosx;

    state_ptr->acc_forward_ms2 = imu_ptr->accel_ms2[Y] * rot_matrix[0][0] + imu_ptr->accel_ms2[X] * rot_matrix[1][0] + imu_ptr->accel_ms2[Z] * rot_matrix[2][0];
    state_ptr->acc_right_ms2 = imu_ptr->accel_ms2[Y] * rot_matrix[0][1] + imu_ptr->accel_ms2[X] * rot_matrix[1][1] + imu_ptr->accel_ms2[Z] * rot_matrix[2][1];
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * rot_matrix[0][2] + imu_ptr->accel_ms2[X] * rot_matrix[1][2] + imu_ptr->accel_ms2[Z] * rot_matrix[2][2]) - 9.806f;

    //printf("%.1f,%.1f,%.1f\n", state_ptr->acc_forward_ms2, state_ptr->acc_right_ms2, state_ptr->acc_up_ms2);
}



void predict_altitude_velocity()
{
    state_ptr->vel_up_ms += (state_ptr->acc_up_ms2 - acc_z_bias) * 0.002f;
    state_ptr->altitude_m += state_ptr->vel_up_ms * 0.002f + (state_ptr->acc_up_ms2 - acc_z_bias) * 0.000002f;
}

void correct_altitude_velocity()
{
    static float diff_alt = 0.0f, diff_vel = 0.0f;

    diff_vel = state_ptr->vel_up_ms - baro_ptr->velocity_ms;
    diff_alt = state_ptr->altitude_m - baro_ptr->altitude_m;

    state_ptr->vel_up_ms -= diff_vel * 0.018f;
    state_ptr->altitude_m -= diff_alt * 0.018f;

    acc_z_bias += diff_vel * 0.0008f;
}
















// ################################################################################################################################
/*
config_t *config = NULL;

void ahrs_init(icm42688p_t *imu, states_t *state, config_t *cfg)
{
    float acc_vector = imu->accel_ms2[X] * imu->accel_ms2[X] + imu->accel_ms2[Y] * imu->accel_ms2[Y] + imu->accel_ms2[Z] * imu->accel_ms2[Z];
    state->pitch_deg = (asinf(imu->accel_ms2[Y] / sqrtf(acc_vector))) * RAD_TO_DEG;
    state->roll_deg = -atan2f(imu->accel_ms2[X] , imu->accel_ms2[Z]) * RAD_TO_DEG;
    state->heading_deg = 0;
    config = cfg;
}

void ahrs_predict(icm42688p_t *imu, states_t *state)
{
    static const float update_rate = 1000.0f;

    state->pitch_deg += imu->gyro_dps[X] / update_rate;
    state->roll_deg += imu->gyro_dps[Y] / update_rate;
    state->heading_deg += -imu->gyro_dps[Z] / update_rate;

    if (state->heading_deg < 0) state->heading_deg += 360.0f;
    else if (state->heading_deg >= 360.0f) state->heading_deg -= 360.0f;

    float sin_yaw = sinf(-imu->gyro_dps[Z] / update_rate * DEG_TO_RAD);

    state->pitch_deg -= state->roll_deg * sin_yaw;
    state->roll_deg += state->pitch_deg * sin_yaw;

    state->pitch_dps = imu->gyro_dps[X];
    state->roll_dps = imu->gyro_dps[Y];
    state->yaw_dps = -imu->gyro_dps[Z];
}


void ahrs_correct(icm42688p_t *imu, states_t *state)
{
    static const float update_rate = 500.0f;
    static float acc_vector = 0;
    static float pitch_acc_deg = 0;
    static float roll_acc_deg = 0;
    static float diff_pitch = 0;
    static float diff_roll = 0;

    acc_vector = imu->accel_ms2[X] * imu->accel_ms2[X] + imu->accel_ms2[Y] * imu->accel_ms2[Y] + imu->accel_ms2[Z] * imu->accel_ms2[Z];

    pitch_acc_deg = (asinf(imu->accel_ms2[Y] / sqrtf(acc_vector))) * RAD_TO_DEG;
    roll_acc_deg = -atan2f(imu->accel_ms2[X] , imu->accel_ms2[Z]) * RAD_TO_DEG;

    if (pitch_acc_deg == NAN) pitch_acc_deg = state->pitch_deg;
    if (roll_acc_deg == NAN) roll_acc_deg = state->roll_deg;

    diff_pitch = state->pitch_deg - pitch_acc_deg;
    diff_roll = state->roll_deg - roll_acc_deg;

    state->pitch_deg -= diff_pitch * config->ahrs_filter_beta / update_rate;
    state->roll_deg -= diff_roll * config->ahrs_filter_beta / update_rate;

    imu->gyro_bias_dps[X] += diff_pitch * config->ahrs_filter_zeta / update_rate;
    imu->gyro_bias_dps[Y] += diff_roll * config->ahrs_filter_zeta / update_rate;

    state->pitch_dps = imu->gyro_dps[X];
    state->roll_dps = imu->gyro_dps[Y];
    state->yaw_dps = -imu->gyro_dps[Z];
}
*/
// ################################################################################################################################
