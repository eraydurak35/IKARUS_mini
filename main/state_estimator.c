#include "state_estimator.h"
#include "bmp390.h"
#include "math.h"
#include "comminication.h"
#include "quaternion.h"
#include "esp_timer.h"

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
static imu_t *imu_ptr = NULL;
static magnetometer_t *mag_ptr = NULL;
static bmp390_t *baro_ptr = NULL;
static config_t *config_ptr = NULL;

static float acc_up_bias = 0;


static float state_velocity_z = 0;
static float filtered_Z = 0;
static float state_estimated_Z = 0;



// Kalman filtre parametreleri
typedef struct {
    float x;   // Durum (yükseklik)
    float v;   // Hız
    float P[2][2];  // Durum kovaryans matrisi
    float Q[2][2];  // Süreç kovaryans matrisi
    float R;        // Ölçüm kovaryans matrisi (barometrik sensör)
} KalmanFilter;

KalmanFilter kf;

static void get_attitude_heading();
static float deadband(float value, const float threshold);
void predict_altitude();
void update_altitude();
void kalman_init(KalmanFilter *kf, float initial_height, float initial_velocity, float process_noise, float measurement_noise);
void kalman_predict(KalmanFilter *kf, float measured_acceleration, float dt);
void kalman_update(KalmanFilter *kf, float measured_height);

void ahrs_init(config_t *cfg, states_t *sta, imu_t *icm, magnetometer_t *hmc, bmp390_t *baro)
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

    kalman_init(&kf, 0, 0, 0.001, 10.0);
}


void ahrs_predict()
{
    static uint8_t init = 1;
    static int64_t t = 0;
    if (init == 1)
    {
        t = esp_timer_get_time();
        init = 0;
    }
    float dt = ((esp_timer_get_time() - t)) / 1000000.0f;
    t = esp_timer_get_time();

    gyr_vec.x = imu_ptr->gyro_dps[Y] * DEG_TO_RAD;
    gyr_vec.y = imu_ptr->gyro_dps[X] * DEG_TO_RAD;
    gyr_vec.z = -imu_ptr->gyro_dps[Z] * DEG_TO_RAD;

    gyr_vec.x -= (config_ptr->ahrs_filter_beta * err.x);
    gyr_vec.y -= (config_ptr->ahrs_filter_beta * err.y);
    gyr_vec.z -= (config_ptr->ahrs_filter_beta * err.z);

    get_quat_deriv(&q, &gyr_vec, &q_dot);

    q.w += q_dot.w * dt;
    q.x += q_dot.x * dt;
    q.y += q_dot.y * dt;
    q.z += q_dot.z * dt;

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
/*  static float rot_matrix[3][3] = {0};
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
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * rot_matrix[0][2] + imu_ptr->accel_ms2[X] * rot_matrix[1][2] + imu_ptr->accel_ms2[Z] * rot_matrix[2][2]) - 9.906f; */


    static float rot_matrix[3][3] = {0};


/*     rot_matrix[0][0] = (q.w * q.w) + (q.x * q.x) - (q.y * q.y) - (q.z * q.z);
    rot_matrix[0][1] = 2.0f * (q.x * q.y - q.w * q.z);
    rot_matrix[0][2] = 2.0f * (q.x * q.z + q.w * q.y);

    rot_matrix[1][0] = 2.0f * (q.x * q.y + q.w * q.z);
    rot_matrix[1][1] = (q.w * q.w) - (q.x * q.x) + (q.y * q.y) - (q.z * q.z);
    rot_matrix[1][2] = 2.0f * (q.y * q.z - q.w * q.x); */

    rot_matrix[2][0] = 2.0f * (q.x * q.z - q.w * q.y);
    rot_matrix[2][1] = 2.0f * (q.w * q.x + q.y * q.z);
    rot_matrix[2][2] = (q.w * q.w) - (q.x * q.x) - (q.y * q.y) + (q.z * q.z);


/*     state_ptr->acc_right_ms2 = -(imu_ptr->accel_ms2[Y] * -rot_matrix[0][0] + imu_ptr->accel_ms2[X] * -rot_matrix[0][1] + imu_ptr->accel_ms2[Z] * rot_matrix[0][2]);
    state_ptr->acc_forward_ms2 = (imu_ptr->accel_ms2[Y] * -rot_matrix[1][0] + imu_ptr->accel_ms2[X] * -rot_matrix[1][1] + imu_ptr->accel_ms2[Z] * rot_matrix[1][2]); */
    state_ptr->acc_up_ms2 = (imu_ptr->accel_ms2[Y] * -rot_matrix[2][0] + imu_ptr->accel_ms2[X] * -rot_matrix[2][1] + imu_ptr->accel_ms2[Z] * rot_matrix[2][2]) - 9.906f;

    
    //printf("%.2f\n", state_ptr->acc_up_ms2);


}



void predict_altitude_velocity()
{
    predict_altitude();

    static uint8_t init = 1;
    static int64_t t = 0; 
    if (init == 1)
    {
        t = esp_timer_get_time();
        init = 0;
    }

    float dt = ((esp_timer_get_time() - t)) / 1000000.0f;
    t = esp_timer_get_time();

    kalman_predict(&kf, state_ptr->acc_up_ms2, dt);


/*     state_ptr->altitude_m += state_ptr->vel_up_ms * dt + (state_ptr->acc_up_ms2 - acc_up_bias) * (dt * dt) * 0.5f;
    state_ptr->vel_up_ms += (state_ptr->acc_up_ms2 - acc_up_bias) * dt; */
   
}

void correct_altitude_velocity()
{
    kalman_update(&kf, baro_ptr->altitude_m);
    //update_altitude();
/*     static float diff_alt = 0.0f, diff_vel = 0.0f;

    diff_vel = baro_ptr->velocity_ms - state_ptr->vel_up_ms;
    diff_alt = baro_ptr->altitude_m - state_ptr->altitude_m;

    state_ptr->vel_up_ms += diff_vel * 0.0109f;
    state_ptr->altitude_m += diff_alt * 0.0151f; */

    //printf("%.2f,%.2f\n", state_ptr->altitude_m, baro_ptr->altitude_m);
    //printf("%.2f,%.2f\n", state_ptr->vel_up_ms, baro_ptr->velocity_ms);
    //printf("%.2f\n", state_ptr->acc_up_ms2);

/*     acc_up_bias -= diff_vel * 0.0001f;

    if (acc_up_bias > 0.05f) acc_up_bias = 0.05f;
    else if (acc_up_bias < -0.05f) acc_up_bias = -0.05f; */

/*     printf("%.3f\n", acc_up_bias); */
}



void calculate_altitude_velocity() // 500Hz
{
    static uint8_t init = 1;
    static float baro_alt_offset_m;
    static float baro_altitude_m;
    static float range_altitude_m = -1;
    static float prev_baro_altitude_m;
    float rangefinder_transition_gain;
    float velocity_accel_ms;
    static float velocity_ms;
    static float accel_altitude_m;
    float baro_velocity_ms;
    static const float dt = 0.002f;
    static float upsampled_baro_altitude_m;
    static float upsampled_range_altitude_m;

    upsampled_baro_altitude_m += (baro_ptr->altitude_m - upsampled_baro_altitude_m) * 0.1f; // updates 50 Hz
    //upsampled_range_altitude_m += ((range_ptr->range_cm / 100.0f) - upsampled_range_altitude_m) * 0.1f; // updates 100 Hz

    upsampled_range_altitude_m = -1;

    baro_altitude_m = upsampled_baro_altitude_m;

    if (upsampled_range_altitude_m > RANGE_BARO_TRANS_END_ALT)
        range_altitude_m = -1.0;

    else range_altitude_m = upsampled_range_altitude_m;

    if (range_altitude_m >= 0 && range_altitude_m < RANGE_BARO_TRANS_START_ALT)
    {
        baro_alt_offset_m = baro_altitude_m - range_altitude_m;
        baro_altitude_m = range_altitude_m;
    }
    else
    {
        baro_altitude_m -= baro_alt_offset_m;
        if (range_altitude_m > 0)
        {
            rangefinder_transition_gain = (RANGE_BARO_TRANS_END_ALT - range_altitude_m) * (RANGE_BARO_TRANS_END_ALT / (RANGE_BARO_TRANS_END_ALT - RANGE_BARO_TRANS_START_ALT));
            baro_altitude_m = range_altitude_m * rangefinder_transition_gain + baro_altitude_m * (1.0f - rangefinder_transition_gain);
        }
    }

    velocity_accel_ms = (state_ptr->acc_up_ms2 - acc_up_bias) * dt;
    accel_altitude_m += (velocity_accel_ms * 0.5f) * dt + velocity_ms * dt;
    accel_altitude_m = accel_altitude_m * config_ptr->alt_filter_beta + baro_altitude_m * (1.0f - config_ptr->alt_filter_beta);
    velocity_ms += velocity_accel_ms;

    if (range_altitude_m >= 0 && range_altitude_m < RANGE_BARO_TRANS_START_ALT)
    {
        state_ptr->altitude_m = baro_altitude_m;
    }
    else
    {
        state_ptr->altitude_m = accel_altitude_m;
    }

    if (init == 1)
    {
        prev_baro_altitude_m = baro_altitude_m;
        init = 0;
    }

    baro_velocity_ms = (baro_altitude_m - prev_baro_altitude_m) / dt;
    prev_baro_altitude_m = baro_altitude_m;

    float velDiff;
    velDiff = baro_velocity_ms - velocity_ms;

    if (velDiff > 8.0f) velDiff = 8.0f;
    else if (velDiff < -8.0f) velDiff = -8.0f;

    velocity_ms += velDiff * config_ptr->velz_filter_beta * dt;
    acc_up_bias -= velDiff * config_ptr->velz_filter_zeta * dt * dt;

    state_ptr->vel_up_ms = velocity_ms;

/*     printf("%.2f, %.2f\n", baro_velocity_ms, state_ptr->vel_up_ms); */

}






void predict_altitude()
{
    static uint8_t init = 1;
    static int64_t t = 0; 
    if (init == 1)
    {
        t = esp_timer_get_time();
        init = 0;
    }
    float dt = ((esp_timer_get_time() - t)) / 1000000.0f;
    t = esp_timer_get_time();

    state_velocity_z += deadband(state_ptr->acc_up_ms2, 0.3f) * dt;
    state_velocity_z *= 0.995f;
}

void update_altitude()
{
    static uint8_t init = 1;
    static int64_t t = 0; 
    if (init == 1)
    {
        t = esp_timer_get_time();
        init = 0;
    }
    float dt = ((esp_timer_get_time() - t)) / 1000000.0f;
    t = esp_timer_get_time();



    filtered_Z = (0.92f) * state_estimated_Z + (1.0f - 0.92f) * baro_ptr->altitude_m;
    state_estimated_Z = filtered_Z + (state_velocity_z * dt);

    printf("%.2f,%.2f\n", state_estimated_Z, baro_ptr->altitude_m);

}



static float deadband(float value, const float threshold)
{
    if (fabsf(value) < threshold)
    {
        value = 0;
    }
    else if (value > 0)
    {
        value -= threshold;
    }
    else if (value < 0)
    {
        value += threshold;
    }
    return value;
}



// Kalman filtresi başlatma fonksiyonu
void kalman_init(KalmanFilter *kf, float initial_height, float initial_velocity, float process_noise, float measurement_noise) 
{
    kf->x = initial_height;
    kf->v = initial_velocity;
    kf->P[0][0] = 1;
    kf->P[0][1] = 0;
    kf->P[1][0] = 0;
    kf->P[1][1] = 1;
    kf->Q[0][0] = process_noise;
    kf->Q[0][1] = 0;
    kf->Q[1][0] = 0;
    kf->Q[1][1] = process_noise;
    kf->R = measurement_noise;
}

// Predict adımı
void kalman_predict(KalmanFilter *kf, float measured_acceleration, float dt) 
{
    // Ön tahmin
    kf->x = kf->x + kf->v * dt + 0.5f * measured_acceleration * dt * dt;
    kf->v = kf->v + measured_acceleration * dt;

    state_ptr->altitude_m = kf->x;
    state_ptr->vel_up_ms = kf->v;

    // Kovaryans matrisi güncellemesi
    kf->P[0][0] += dt * (2.0f * kf->P[0][1] + dt * kf->P[1][1]) + kf->Q[0][0];
    kf->P[0][1] += dt * kf->P[1][1];
    kf->P[1][0] += dt * kf->P[1][1];
    kf->P[1][1] += kf->Q[1][1];
}

// Update adımı
void kalman_update(KalmanFilter *kf, float measured_height) 
{
    // Ölçüm yenilemesi
    float y = measured_height - kf->x; // Ölçüm yeniliği
    float S = kf->P[0][0] + kf->R; // Yenilik kovaryansı
    float K[2]; // Kalman kazancı
    K[0] = kf->P[0][0] / S;
    K[1] = kf->P[1][0] / S;

    // Durum vektörü güncellemesi
    kf->x += K[0] * y;
    kf->v += K[1] * y;

    //printf("%.2f,%.2f\n", kf->x, baro_ptr->altitude_m);
    //printf("%.2f,%.2f\n", kf->v, baro_ptr->velocity_ms);

    //printf("%.4f,%.4f\n", K[0], K[1]);

    // Kovaryans matrisi güncellemesi
    float P00_temp = kf->P[0][0];
    float P01_temp = kf->P[0][1];
    kf->P[0][0] -= K[0] * P00_temp;
    kf->P[0][1] -= K[0] * P01_temp;
    kf->P[1][0] -= K[1] * P00_temp;
    kf->P[1][1] -= K[1] * P01_temp;
}






