#include <stdio.h>
#include "control_algorithm.h"
#include "comminication.h"
#include "state_estimator.h"
#include "gpio.h"
#include "math.h"
#include "filters.h"
#include "typedefs.h"
#include "esp_timer.h"

static gamepad_t prv_gamepad;
static gamepad_t *gamepad_p;
static flight_t *flight_p;
static target_t *target_p;
static states_t *state_p;
static telemetry_small_t *telemetry_p;
static config_t *config_p;

static biquad_lpf_t lpf_pitch_d_term;
static biquad_lpf_t lpf_roll_d_term;

typedef struct
{
    float pitch;
    float roll;
} flip_vec_t;

static flip_vec_t flip_vec;

struct PID
{
  float errPitch, errRoll, errYaw;
  float errPitchPrev, errRollPrev, errYawPrev;
  float pitchDegsPrev, rollDegsPrev, yawDegsPrev;
  float pitchPout, rollPout, yawPout;
  float pitchIout, rollIout, yawIout;
  float pitchDout, rollDout, yawDout;
  float pitchPIDout, rollPIDout, yawPIout;
  float errVel_x, errVel_x_prev, velocity_x_ms_prev;
  float errVel_y, errVel_y_prev, velocity_y_ms_prev;
  float errVel_z, errVel_z_prev, velocity_z_ms_prev;
  float altPout, altIout, altDout;
  float posXPout, posXIout;
  float posYPout, posYIout;
  float acc_z_prev;
};


static float arm_heading = 0;

// 0 -> direction_not_chosen
// 1 -> flip_is_ongoing
// 2 -> flip_done (stick should return to center to reset to direction_not_chosen)
static uint8_t flip_status = 0;
static float pitch_flip_integrator_deg = 0;
static float roll_flip_integrator_deg = 0;
static float flip_up_velocity_integrator = 0;

const float maxI = 200.0f, maxPID = 400.0f;
const float thr_min = 25, thr_max = 1023;
float thr_m1 = 0, thr_m2 = 0, thr_m3 = 0, thr_m4 = 0;
struct PID pid;
static float applyExpo(int input);
static void outer_control_loop(float dt);
static void inner_control_loop();
static void arm();
static void disarm();

void control_init(gamepad_t *gamepad, telemetry_small_t *telemetry, flight_t *flight, target_t *target, states_t *state, config_t *config)
{
    gamepad_p = gamepad;
    flight_p = flight;
    target_p = target;
    state_p = state;
    telemetry_p = telemetry;
    config_p = config;

    biquad_lpf_configure(50.0f, &lpf_pitch_d_term);
    biquad_lpf_configure(50.0f, &lpf_roll_d_term);
}


void check_flight_mode()
{
    if (gamepad_p->analog_LX > 108 && gamepad_p->analog_LY < -108 && !flight_p->arm_status && telemetry_p->rssi < 0) arm();
    else if ((gamepad_p->analog_LX < -108 && gamepad_p->analog_LY < -108 && flight_p->arm_status) || telemetry_p->rssi == 0) disarm();

    if (gamepad_p->button_A == 1 && telemetry_p->is_headless_on == 0) telemetry_p->is_headless_on = 1;
    else if (gamepad_p->button_A == 0 && telemetry_p->is_headless_on == 1) telemetry_p->is_headless_on = 0;

    if (gamepad_p->button_B == 1 && prv_gamepad.button_B == 0) telemetry_p->is_flip_on = 1;
    else if (gamepad_p->button_B == 0) telemetry_p->is_flip_on = 0;

    if (gamepad_p->button_X == 1 && telemetry_p->is_alt_hold_on == 0)
    {
        telemetry_p->is_alt_hold_on = 1;
        pid.altIout = target_p->throttle;
        target_p->altitude = state_p->altitude_m;
    }
    else if (gamepad_p->button_X == 0 && telemetry_p->is_alt_hold_on == 1) telemetry_p->is_alt_hold_on = 0;

    prv_gamepad = *gamepad_p;
}


static void arm()
{
    flip_status = 0;
    target_p->throttle = IDLE_THROTTLE;
    pid.pitchDegsPrev = state_p->pitch_dps;
    pid.rollDegsPrev = state_p->roll_dps;
    pid.yawDegsPrev = state_p->yaw_dps;

    pid.velocity_x_ms_prev = state_p->vel_forward_ms;
    pid.velocity_y_ms_prev = state_p->vel_right_ms;
    pid.velocity_z_ms_prev = state_p->vel_up_ms;

    target_p->pitch_deg = 0;
    target_p->pitch_dps = 0;

    target_p->roll_deg = 0;
    target_p->roll_dps = 0;

    target_p->heading_deg = state_p->heading_deg;
    target_p->yaw_dps = 0;

    target_p->altitude = 0;
    target_p->velocity_x_ms = 0;
    target_p->velocity_y_ms = 0;
    target_p->velocity_z_ms = 0;

    pid.pitchIout = 0;
    pid.rollIout = 0;
    pid.yawIout = 0;
    pid.altIout = 0;
    pid.posXIout = 0;
    pid.posYIout = 0;

    pid.errPitchPrev = 0;
    pid.errRollPrev = 0;
    pid.errYawPrev = 0;

    state_p->altitude_m = 0.0f;
    state_p->vel_up_ms = 0.0f;

    arm_heading = state_p->heading_deg;
    flight_p->arm_status = 1;
}

static void disarm()
{
    set_throttle(0, 0, 0, 0);
    //telemetry_p->arm_status = 0;
    flight_p->arm_status = 0;
}


void flight_control()
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

    
    if (flight_p->arm_status == 1)
    {
        outer_control_loop(dt);
        inner_control_loop();
    }
    else
    {
        disarm();
    }
}

static void outer_control_loop(float dt)
{

    if (flip_status > 0)
    {
        flip_up_velocity_integrator += state_p->acc_up_ms2 * dt;
    }

    static uint8_t counter = 0;
    static float inner_dt = 0;
    inner_dt += dt;
    counter++;

    if (counter >= 5) // 100Hz
    {
        counter = 0;

        if (telemetry_p->is_flip_on == 1 && flip_status == 0)
        {
            if ((gamepad_p->analog_RX > 110 || gamepad_p->analog_RX < -110) || (gamepad_p->analog_RY > 110 || gamepad_p->analog_RY < -110))
            {

                if (gamepad_p->analog_RX > 110)
                {
                    flip_vec.pitch = 0.0f;
                    flip_vec.roll = 380.0f;
                }
                else if (gamepad_p->analog_RX < -110)
                {
                    flip_vec.pitch = 0.0f;
                    flip_vec.roll = -380.0f;
                }
                else if (gamepad_p->analog_RY > 110)
                {
                    flip_vec.pitch = -380.0f;
                    flip_vec.roll = 0.0f;
                }
                else if (gamepad_p->analog_RY < -110)
                {
                    flip_vec.pitch = 380.0f;
                    flip_vec.roll = 0.0f;
                }

                flip_up_velocity_integrator = 0;
                pitch_flip_integrator_deg = 0;
                roll_flip_integrator_deg = 0;
                flip_status = 1;
            }
        }
        

        if (flip_status == 1)
        {
            target_p->pitch_dps = 0;
            target_p->roll_dps = 0;
            target_p->throttle = 900.0f;
            if (flip_up_velocity_integrator > 1.7f)
            {
                flip_status = 2;
            }
        }

        else if (flip_status == 2)
        {
            target_p->throttle = IDLE_THROTTLE;
            roll_flip_integrator_deg += state_p->roll_dps * inner_dt;
            pitch_flip_integrator_deg += state_p->pitch_dps * inner_dt;


            target_p->roll_dps = (flip_vec.roll - roll_flip_integrator_deg) * 10.0f;
            if (target_p->roll_dps > 700.0f) target_p->roll_dps = 700.0f;
            else if (target_p->roll_dps < -700.0f) target_p->roll_dps = -700.0f;


            target_p->pitch_dps = (flip_vec.pitch - pitch_flip_integrator_deg) * 10.0f;
            if (target_p->pitch_dps > 700.0f) target_p->pitch_dps = 700.0f;
            else if (target_p->pitch_dps < -700.0f) target_p->pitch_dps = -700.0f;

            float vec_len = sqrtf((pitch_flip_integrator_deg * pitch_flip_integrator_deg) + (roll_flip_integrator_deg * roll_flip_integrator_deg));
            if (vec_len >= 365.0f)
            {
                flip_status = 3;
            }
        }
        else if (flip_status == 3)
        {
            target_p->throttle = 900.0f;

            if (flip_up_velocity_integrator >= 0.2f)
            {
                telemetry_p->is_flip_on = 0;
                flip_status = 0;
            }
        }

        if (flip_status == 0 || flip_status == 3) // flip is not ongoing
        {

            target_p->pitch_deg = -gamepad_p->analog_RY * (config_p->max_pitch_angle / 120.0f);
            target_p->roll_deg = gamepad_p->analog_RX * (config_p->max_roll_angle / 120.0f);

            if (telemetry_p->is_headless_on == 1.0f)
            {
                float head_diff = (state_p->heading_deg - arm_heading) * DEG_TO_RAD;
                float temp_pitch = target_p->pitch_deg;
                float temp_roll = target_p->roll_deg;
                target_p->pitch_deg = temp_pitch * cosf(head_diff) - temp_roll * sinf(head_diff);
                target_p->roll_deg = temp_pitch * sinf(head_diff) + temp_roll * cosf(head_diff);
            }
        }

        if (flip_status == 0 || flip_status == 3)
        {
            /////////////////////   Pitch Rate Controller   ///////////////////////
            target_p->pitch_dps = (target_p->pitch_deg - state_p->pitch_deg) * config_p->pitch_rate_scal;
            if (target_p->pitch_dps > config_p->max_pitch_rate) target_p->pitch_dps = config_p->max_pitch_rate;
            else if (target_p->pitch_dps < -config_p->max_pitch_rate) target_p->pitch_dps = -config_p->max_pitch_rate;

            /////////////////////   Roll Rate Controller   ///////////////////////
            target_p->roll_dps = (target_p->roll_deg - state_p->roll_deg) * config_p->roll_rate_scal;
            if (target_p->roll_dps > config_p->max_roll_rate) target_p->roll_dps = config_p->max_roll_rate;
            else if (target_p->roll_dps < -config_p->max_roll_rate) target_p->roll_dps = -config_p->max_roll_rate;


            /////////////////////   Yaw Rate Controller   ///////////////////////
            // Yaw stick controls yaw rate
            //if ((gamepad_p->analog_LX < 250 && gamepad_p->analog_LX > -250) || gamepad_p->analog_LY < -700) // Yaw stick is centered
            if ((gamepad_p->analog_LX < 30 && gamepad_p->analog_LX > -30) || gamepad_p->analog_LY < -90) // Yaw stick is centered
            {
                // Calculate yaw rate from setpoint error
                target_p->yaw_dps = (target_p->heading_deg - state_p->heading_deg) * config_p->yaw_rate_scale;

                // This part ensures the craft turns from closest side to setpoint
                // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
                // If we dont do this craft will attempt to turn counter clockwise 265deg
                if (target_p->yaw_dps < -180.0 * config_p->yaw_rate_scale) target_p->yaw_dps += 360.0 * config_p->yaw_rate_scale;
                else if (target_p->yaw_dps > 180.0 * config_p->yaw_rate_scale) target_p->yaw_dps -= 360.0 * config_p->yaw_rate_scale;
            }
            else // Yaw stick is not centered
            {
                /* target_p->yaw_dps = (gamepad_p->analog_LX / 1000.0f) * config_p->max_yaw_rate; */
                target_p->yaw_dps = (gamepad_p->analog_LX / 120.0f) * config_p->max_yaw_rate;
                target_p->heading_deg = state_p->heading_deg;
            }
            // Just limit the yaw rate so it doesnt go nuts
            if (target_p->yaw_dps > config_p->max_yaw_rate) target_p->yaw_dps = config_p->max_yaw_rate;
            else if (target_p->yaw_dps < -config_p->max_yaw_rate) target_p->yaw_dps = -config_p->max_yaw_rate;
        }
        else
        {
            target_p->yaw_dps = 0;
        }


        if (telemetry_p->is_alt_hold_on == 1)
        {
            ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
            // Throttle controls the altitude velocity
            if (gamepad_p->analog_LY < 30 && gamepad_p->analog_LY > -30) // Throttle stick is centered, velocity calculated from setpoint error
            {
                // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
                // we always aim 0.1m higher to make sure target is always reached
                target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) * config_p->alt_vel_scale;
                //target.velocity_z_ms = applyDeadband(target.velocity_z_ms, 0.05);
            }
            else // Throttle stick not centered, velocity calculated from stick input
            {
                // Calculate the desired altitude velocity from raw stick input
                target_p->velocity_z_ms = (gamepad_p->analog_LY / 120.0f) * config_p->max_vert_vel;
                // we dont use altitude setpoint if the stick is not centered
                // but we want to set our setpoint when the stick is in middle
                // so that when we let go of the stick, craft stays at the altitude we let go
                target_p->altitude = state_p->altitude_m;
            }
        }
        else
        {
            if (flip_status == 0)
            {
                // I added some throttle curve
                //target_p->throttle += pow((gamepad_p->analog_LY / 1000.0f), 3) * 1.25f;
                target_p->throttle = applyExpo((gamepad_p->analog_LY + 120) / 2) * 5.0f;
                if (target_p->throttle > MAX_TARGET_THROTTLE) target_p->throttle = MAX_TARGET_THROTTLE;
                else if (target_p->throttle < IDLE_THROTTLE) target_p->throttle = IDLE_THROTTLE;
            }

        }
        inner_dt = 0;
    }

}

static void inner_control_loop()
{
    //↓↓↓↓↓↓↓↓↓↓   CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
    float pitch_dps_corrected;
    float roll_dps_corrected;
    float yaw_dps_corrected;

    pitch_dps_corrected = sinf(state_p->roll_deg * DEG_TO_RAD) * target_p->yaw_dps + target_p->pitch_dps;
    roll_dps_corrected = sinf(-state_p->pitch_deg * DEG_TO_RAD) * target_p->yaw_dps + target_p->roll_dps;
    yaw_dps_corrected = fabs(cosf(state_p->roll_deg * DEG_TO_RAD)) * fabs(cosf(state_p->pitch_deg * DEG_TO_RAD)) * target_p->yaw_dps;

    pid.errPitch = pitch_dps_corrected - state_p->pitch_dps;
    pid.errRoll = roll_dps_corrected - state_p->roll_dps;
    pid.errYaw = yaw_dps_corrected - state_p->yaw_dps;
    //↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   PITCH P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPout = pid.errPitch * config_p->pitch_p;
    //↑↑↑↑↑↑↑↑↑↑   PITCH P CALCULATION   ↑↑↑↑↑↑↑↑↑↑


    //↓↓↓↓↓↓↓↓↓↓   ROLL P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPout = pid.errRoll * config_p->roll_p;
    //↑↑↑↑↑↑↑↑↑↑   ROLL P CALCULATION   ↑↑↑↑↑↑↑↑↑↑


    //↓↓↓↓↓↓↓↓↓↓   YAW P CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPout = pid.errYaw * config_p->yaw_p;
    //↑↑↑↑↑↑↑↑↑↑   YAW P CALCULATION   ↑↑↑↑↑↑↑↑↑↑


    if (target_p->throttle > 200 && (flip_status == 0 || flip_status == 3))
    {
        //↓↓↓↓↓↓↓↓↓↓  PITCH I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.pitchIout += config_p->pitch_i * 0.00125f * (pid.errPitch + pid.errPitchPrev);  //  0.00125 = 0.5 * sampleTime
        if (pid.pitchIout > maxI) pid.pitchIout = maxI;
        else if (pid.pitchIout < -maxI) pid.pitchIout = -maxI;
        //↑↑↑↑↑↑↑↑↑↑   PITCH I CALCULATION   ↑↑↑↑↑↑↑↑↑↑


        //↓↓↓↓↓↓↓↓↓↓   ROLL I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.rollIout += config_p->roll_i * 0.00125f * (pid.errRoll + pid.errRollPrev);
        if (pid.rollIout > maxI) pid.rollIout = maxI;
        else if (pid.rollIout < -maxI) pid.rollIout = -maxI;
        //↑↑↑↑↑↑↑↑↑↑   ROLL I CALCULATION   ↑↑↑↑↑↑↑↑↑↑


        //↓↓↓↓↓↓↓↓↓↓   YAW I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.yawIout += config_p->yaw_i * 0.00125f * (pid.errYaw + pid.errYawPrev);
        if (pid.yawIout > maxI) pid.yawIout = maxI;
        else if (pid.yawIout < -maxI) pid.yawIout = -maxI;
        //↑↑↑↑↑↑↑↑↑↑   YAW I CALCULATION   ↑↑↑↑↑↑↑↑↑↑
    }
    else
    {
        pid.pitchIout = 0.0;
        pid.rollIout = 0.0;
        pid.yawIout = 0.0;
    }

    //↓↓↓↓↓↓↓↓↓↓   PITCH D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchDout = -config_p->pitch_d * (state_p->pitch_dps - pid.pitchDegsPrev);
    biquad_lpf(&lpf_pitch_d_term, &pid.pitchDout);
    //↑↑↑↑↑↑↑↑↑↑   PITCH D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   ROLL D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.rollDout = -config_p->roll_d * (state_p->roll_dps - pid.rollDegsPrev);
    biquad_lpf(&lpf_roll_d_term, &pid.rollDout);
    //↑↑↑↑↑↑↑↑↑↑   ROLL D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   PITCH PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.pitchPIDout = pid.pitchPout + pid.pitchIout + pid.pitchDout;
    if (flip_status == 0)
    {
        if (pid.pitchPIDout > maxPID) pid.pitchPIDout = maxPID;
        else if (pid.pitchPIDout < -maxPID) pid.pitchPIDout = -maxPID;
    }

    //↑↑↑↑↑↑↑↑↑↑   PITCH PID OUT   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   ROLL PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.rollPIDout = pid.rollPout + pid.rollIout + pid.rollDout;
    if (flip_status == 0)
    {
        if (pid.rollPIDout > maxPID) pid.rollPIDout = maxPID;
        else if (pid.rollPIDout < -maxPID) pid.rollPIDout = -maxPID;
    }

    //↑↑↑↑↑↑↑↑↑↑   ROLL PID OUT   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   YAW PID OUT   ↓↓↓↓↓↓↓↓↓↓
    pid.yawPIout = pid.yawPout + pid.yawIout;
    if (pid.yawPIout > maxPID) pid.yawPIout = maxPID;
    else if (pid.yawPIout < -maxPID) pid.yawPIout = -maxPID;
    //↑↑↑↑↑↑↑↑↑↑   YAW PID OUT   ↑↑↑↑↑↑↑↑↑↑


    //↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
    pid.errPitchPrev = pid.errPitch;
    pid.errRollPrev = pid.errRoll;
    pid.errYawPrev = pid.errYaw;
    pid.pitchDegsPrev = state_p->pitch_dps;
    pid.rollDegsPrev = state_p->roll_dps;
    pid.yawDegsPrev = state_p->yaw_dps;
    //↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑


    //=============================================================================//
    //                              Altitude Controller
    //=============================================================================//

    if (telemetry_p->is_alt_hold_on == 1)
    {

        //↓↓↓↓↓↓↓↓↓↓  CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
        pid.errVel_z = target_p->velocity_z_ms - state_p->vel_up_ms;
        //↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

        //↓↓↓↓↓↓↓↓↓↓  ALTITUDE P CALCULATION  ↓↓↓↓↓↓↓↓↓↓
        pid.altPout = pid.errVel_z * config_p->alt_p;
        //↑↑↑↑↑↑↑↑↑↑  ALTITUDE P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

        //↓↓↓↓↓↓↓↓↓↓  ALTITUDE I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        if (flight_p->takeoff_status == 0)
        {
            pid.altIout += config_p->alt_i * 0.001f * (pid.errVel_z + pid.errVel_z_prev);  //  0.001 = 0.5 * sampleTime
            if (pid.altIout > MAX_TARGET_THROTTLE) pid.altIout = MAX_TARGET_THROTTLE;
            else if (pid.altIout < IDLE_THROTTLE) pid.altIout = IDLE_THROTTLE;
        }
        //↑↑↑↑↑↑↑↑↑↑   ALTITUDE I CALCULATION   ↑↑↑↑↑↑↑↑↑↑

        //↓↓↓↓↓↓↓↓↓↓   ALTITUDE D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.altDout = -config_p->alt_d * (state_p->acc_up_ms2 + pid.acc_z_prev);
        pid.acc_z_prev = state_p->acc_up_ms2;
        //↑↑↑↑↑↑↑↑↑↑   ALTITUDE D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

        //↓↓↓↓↓↓↓↓↓↓   ALTITUDE PID OUT   ↓↓↓↓↓↓↓↓↓↓
        target_p->throttle = pid.altPout + pid.altIout + pid.altDout;
        if (target_p->throttle > MAX_TARGET_THROTTLE) target_p->throttle = MAX_TARGET_THROTTLE;
        else if (target_p->throttle < IDLE_THROTTLE) target_p->throttle = IDLE_THROTTLE;
        //↑↑↑↑↑↑↑↑↑↑   ALTITUDE PID OUT   ↑↑↑↑↑↑↑↑↑↑

        //↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
        pid.errVel_z_prev = pid.errVel_z;
        //↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑
        
    }



    float comp_target_thr = target_p->throttle;
    // polinomial is specific to this drone
    // collect some data while hovering 
    // substract initial throttle value from all data it should start from 0
    // remove begining and end of the data
    // katsayilar = polyfit(batt_v, thr_zero, 2); this is the function for matlab (2 is for second order)
    //float battery_compans_throttle = (-60.9742f * (telemetry_p->battery_voltage * telemetry_p->battery_voltage)) + (195.5156f * telemetry_p->battery_voltage) + 59.9253f;
    //if (battery_compans_throttle < 0.0f) battery_compans_throttle = 0.0f;
    //comp_target_thr += battery_compans_throttle;

    //float cosAngAbs = cosf(fabs(state_p->pitch_deg) * DEG_TO_RAD) * cosf(fabs(state_p->roll_deg) * DEG_TO_RAD);
    //if (cosAngAbs != 0) comp_target_thr += ((1.0f / cosAngAbs) - 1.0f) * comp_target_thr;

    //↓↓↓↓↓↓↓↓↓↓   MOTOR 1 (LEFT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr_m1 = comp_target_thr - pid.pitchPIDout + pid.rollPIDout + pid.yawPIout;
    if (thr_m1 < thr_min) thr_m1 = thr_min;
    else if (thr_m1 > thr_max) thr_m1 = thr_max;
    //↑↑↑↑↑↑↑↑↑↑   MOTOR 1 (LEFT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   MOTOR 2 (LEFT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr_m2 = comp_target_thr + pid.pitchPIDout + pid.rollPIDout  - pid.yawPIout;
    if (thr_m2 < thr_min) thr_m2 = thr_min;
    else if (thr_m2 > thr_max) thr_m2 = thr_max;
    //↑↑↑↑↑↑↑↑↑↑   MOTOR 2 (LEFT TOP)   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   MOTOR 3 (RIGHT BOTTOM)   ↓↓↓↓↓↓↓↓↓↓
    thr_m3 = comp_target_thr - pid.pitchPIDout - pid.rollPIDout - pid.yawPIout;
    if (thr_m3 < thr_min) thr_m3 = thr_min;
    else if (thr_m3 > thr_max) thr_m3 = thr_max;
    //↑↑↑↑↑↑↑↑↑↑   MOTOR 3 (RIGHT BOTTOM)   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   MOTOR 4 (RIGHT TOP)   ↓↓↓↓↓↓↓↓↓↓
    thr_m4 = comp_target_thr + pid.pitchPIDout - pid.rollPIDout + pid.yawPIout;
    if (thr_m4 < thr_min) thr_m4 = thr_min;
    else if (thr_m4 > thr_max) thr_m4 = thr_max;
    //↑↑↑↑↑↑↑↑↑↑   MOTOR 4 (RIGHT TOP)   ↑↑↑↑↑↑↑↑↑↑

    //↓↓↓↓↓↓↓↓↓↓   OUTPUT TO THE MOTORS   ↓↓↓↓↓↓↓↓↓↓
    set_throttle(thr_m1, thr_m2, thr_m3, thr_m4);
    //↑↑↑↑↑↑↑↑↑↑   OUTPUT TO THE MOTORS   ↑↑↑↑↑↑↑↑↑↑
}


// Expo fonksiyonu
static float applyExpo(int input)
{
    // Giriş aralığını 0 ile 120 arasında normalize et (0-1 aralığına)
    float normalizedInput = (((float)input / 120.0f) - 0.5f) * 2.0f;
    normalizedInput = powf(normalizedInput, 3.0f);
    // Normalized değeri tekrar 0 ile 120 aralığına ölçekle
    return ((normalizedInput * 120.0f) + 120.0f) * 0.5f;
}