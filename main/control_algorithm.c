#include <stdio.h>
#include "control_algorithm.h"
#include "comminication.h"
#include "state_estimator.h"
#include "gpio.h"
#include "math.h"

static gamepad_t prv_gamepad;
static gamepad_t *gamepad_p;
static flight_t *flight_p;
static target_t *target_p;
static states_t *state_p;
static telemetry_t *telemetry_p;
static config_t *config_p;


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
  float posXPout, posXIout, posXDout;
  float posYPout, posYIout, posYDout;
  float acc_z_prev;
};

const float maxI = 200.0, maxPID = 400.0;
const float thr_min = 25, thr_max = 1023;
float thr_m1 = 0, thr_m2 = 0, thr_m3 = 0, thr_m4 = 0;
struct PID pid;



void control_init(gamepad_t *gamepad, telemetry_t *telemetry, flight_t *flight, target_t *target, states_t *state, config_t *config)
{
  gamepad_p = gamepad;
  flight_p = flight;
  target_p = target;
  state_p = state;
  telemetry_p = telemetry;
  config_p = config;
}

void check_flight_mode()
{

  if (gamepad_p->right_shoulder != prv_gamepad.right_shoulder && gamepad_p->right_shoulder == 1)
  {
    if (!flight_p->arm_status)
    {
      // RGB_write(0, 255, 0);
      arm();
    }
    else
    {
      // RGB_write(100, 140, 255); // White
      disarm();
    }
  }

  if (gamepad_p->button_A != prv_gamepad.button_A && gamepad_p->button_A == 1)
  {
    if (!flight_p->alt_hold_status)
    {
      flight_p->alt_hold_status = 1;
      pid.altIout = target_p->throttle;
      target_p->altitude = state_p->altitude_m;
      // RGB_write(0, 0, 255);
    }
    else
    {
      flight_p->alt_hold_status = 0;
      // RGB_write(0, 255, 0);
    }
  }

  if (gamepad_p->button_B != prv_gamepad.button_B && gamepad_p->button_B == 1)
  {
    if (!flight_p->pos_hold_status)
    {
      flight_p->pos_hold_status = 1;
      // RGB_write(0, 100, 255);
    }
    else
    {
      flight_p->pos_hold_status = 0;
      // RGB_write(0, 255, 0);
    }
  }

  prv_gamepad = *gamepad_p;
}

void arm()
{
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

  // baro.ground_pressure = baro.pressure;
  // baro.init_temp = baro.temperature;
  state_p->altitude_m = 0.0f;
  state_p->vel_up_ms = 0.0f;

  // accz_bias = -0.06;


  // if altitude hold mode enabled before arming
  // then when armed, automatically takeoff pre determined altitude
  /*   flight.takeoff_status = false;
    if (flight.alt_hold_status)
    {
      flight.takeoff_status = true;
      pid.altIout = IDLE_THROTTLE;
    }
  */
  telemetry_p->arm_status = 1;
  flight_p->arm_status = 1;
}

void disarm()
{
  set_throttle(0, 0, 0, 0);
  telemetry_p->arm_status = 0;
  flight_p->arm_status = 0;
}


void flight_control()
{
  
  outer_control_loop();

  if (flight_p->arm_status == 1)
  {
    inner_control_loop();
  }
  else
  {
    disarm();
  }
}

void outer_control_loop()
{

  static uint8_t counter = 0;
  counter++;

  if (counter >= 5) // 100Hz
  {
    counter = 0;

    if (flight_p->pos_hold_status == 1)
    {
      target_p->velocity_x_ms = gamepad_p->analog_RY * (config_p->max_horizontal_velocity / -1000.0f);
      target_p->velocity_y_ms = gamepad_p->analog_RX * (config_p->max_horizontal_velocity / 1000.0f);

      pid.errVel_x = -(target_p->velocity_x_ms - state_p->vel_forward_ms);
      pid.errVel_y = target_p->velocity_y_ms - state_p->vel_right_ms;

      pid.posXPout = pid.errVel_x * config_p->position_p;
      pid.posYPout = pid.errVel_y * config_p->position_p;


      pid.posXIout += config_p->position_i * 0.005f * (pid.errVel_x + pid.errVel_x_prev);
      if (pid.posXIout > 2.0f) pid.posXIout = 2.0f;
      else if (pid.posXIout < -2.0f) pid.posXIout = -2.0f;

      pid.posYIout += config_p->position_i * 0.005f * (pid.errVel_y + pid.errVel_y_prev);
      if (pid.posYIout > 2.0f) pid.posYIout = 2.0f;
      else if (pid.posYIout < -2.0f) pid.posYIout = -2.0f;


      pid.posXDout = -config_p->position_d * (state_p->vel_forward_ms - pid.velocity_x_ms_prev);
      pid.posYDout = -config_p->position_d * (state_p->vel_right_ms - pid.velocity_y_ms_prev);


      target_p->pitch_deg = pid.posXPout + pid.posXIout + pid.posXDout;
      if (target_p->pitch_deg > config_p->max_pitch_angle) target_p->pitch_deg = config_p->max_pitch_angle;
      else if (target_p->pitch_deg < -config_p->max_pitch_angle) target_p->pitch_deg = -config_p->max_pitch_angle;


      target_p->roll_deg = pid.posYPout + pid.posYIout + pid.posYDout;
      if (target_p->roll_deg > config_p->max_roll_angle) target_p->roll_deg = config_p->max_roll_angle;
      else if (target_p->roll_deg < -config_p->max_roll_angle) target_p->roll_deg = -config_p->max_roll_angle;


    }
    else
    {
      target_p->pitch_deg = gamepad_p->analog_RY * (config_p->max_pitch_angle / 1000.0f);
      target_p->roll_deg = gamepad_p->analog_RX * (config_p->max_roll_angle / 1000.0f);
    }



    /////////////////////   Pitch Rate Controller   ///////////////////////
    target_p->pitch_dps = (target_p->pitch_deg - state_p->pitch_deg) * config_p->pitch_rate_scale;
    if (target_p->pitch_dps > config_p->max_pitch_rate) target_p->pitch_dps = config_p->max_pitch_rate;
    else if (target_p->pitch_dps < -config_p->max_pitch_rate) target_p->pitch_dps = -config_p->max_pitch_rate;

    /////////////////////   Roll Rate Controller   ///////////////////////
    target_p->roll_dps = (target_p->roll_deg - state_p->roll_deg) * config_p->roll_rate_scale;
    if (target_p->roll_dps > config_p->max_roll_rate) target_p->roll_dps = config_p->max_roll_rate;
    else if (target_p->roll_dps < -config_p->max_roll_rate) target_p->roll_dps = -config_p->max_roll_rate;


    /////////////////////   Yaw Rate Controller   ///////////////////////
    // Right Left trigger controls yaw rate
    if (gamepad_p->right_trigger - gamepad_p->left_trigger < 150 && gamepad_p->right_trigger - gamepad_p->left_trigger > -150) // Yaw stick is centered
    {
      // Calculate yaw rate from setpoint error
      target_p->yaw_dps = (target_p->heading_deg - state_p->heading_deg) * config_p->yaw_rate_scale;

      // This part ensures the craft turns from closest side to setpoint
      // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
      // If we dont do this craft will attempt to turn counter clockwise 265deg
      if (target_p->yaw_dps < -180.0f * config_p->yaw_rate_scale) target_p->yaw_dps += 360.0f * config_p->yaw_rate_scale;
      else if (target_p->yaw_dps > 180.0f * config_p->yaw_rate_scale) target_p->yaw_dps -= 360.0f * config_p->yaw_rate_scale;
    }
    else // Yaw stick is not centered
    {
      target_p->yaw_dps = ((gamepad_p->right_trigger - gamepad_p->left_trigger) / 1000.0f) * config_p->max_yaw_rate;
      target_p->heading_deg = state_p->heading_deg;
    }
    // Just limit the yaw rate so it doesnt go nuts
    if (target_p->yaw_dps > config_p->max_yaw_rate) target_p->yaw_dps = config_p->max_yaw_rate;
    else if (target_p->yaw_dps < -config_p->max_yaw_rate) target_p->yaw_dps = -config_p->max_yaw_rate;
/*
    // Yaw stick controls yaw rate
    if (gamepad.analog_LX < 150 && gamepad.analog_LX > -150) // Yaw stick is centered
    {
      // Calculate yaw rate from setpoint error
      target.yaw_degs = (target.heading - states.heading) * config.yaw_rate_scale;

      // This part ensures the craft turns from closest side to setpoint
      // Say the setpoint is 5 deg and craft is at 270, logical thing is craft turns clockwise 95 deg
      // If we dont do this craft will attempt to turn counter clockwise 265deg
      if (target.yaw_degs < -180.0 * config.yaw_rate_scale) target.yaw_degs += 360.0 * config.yaw_rate_scale;
      else if (target.yaw_degs > 180.0 * config.yaw_rate_scale) target.yaw_degs -= 360.0 * config.yaw_rate_scale;
    }
    else // Yaw stick is not centered
    {
      target.yaw_degs = (gamepad.analog_LX / 1000.0f) * config.max_yaw_rate;
      target.heading = states.heading;
    }
    // Just limit the yaw rate so it doesnt go nuts
    if (target.yaw_degs > config.max_yaw_rate) target.yaw_degs = config.max_yaw_rate;
    else if (target.yaw_degs < -config.max_yaw_rate) target.yaw_degs = -config.max_yaw_rate;
*/

    
    if (flight_p->alt_hold_status == 1)
    {
      if (flight_p->takeoff_status == 1)
      {
        pid.altIout += (config_p->hover_throttle - IDLE_THROTTLE) / 100.0f;

        if (pid.altIout >= config_p->hover_throttle)
        {
          target_p->altitude = config_p->takeoff_altitude;
          flight_p->takeoff_status = 0;
        }
      }
      ////////////////////////    Altitude Velocity Controller    /////////////////////////////////
      // Throttle controls the altitude velocity
      if (gamepad_p->analog_LY < 250 && gamepad_p->analog_LY > -250) // Throttle stick is centered, velocity calculated from setpoint error
      {
        // Target velocity is calculated from dividing the difference between set altitude and actual altitude with a constant value
        // we always aim 0.1m higher to make sure target is always reached
        target_p->velocity_z_ms = (target_p->altitude - state_p->altitude_m) / 2.0f;
        //target.velocity_z_ms = applyDeadband(target.velocity_z_ms, 0.05);
      }
      else // Throttle stick not centered, velocity calculated from stick input
      {
        // Calculate the desired altitude velocity from raw stick input
        target_p->velocity_z_ms = (gamepad_p->analog_LY / 1000.0f) * config_p->max_vertical_velocity;
        // we dont use altitude setpoint if the stick is not centered
        // but we want to set our setpoint when the stick is in middle
        // so that when we let go of the stick, craft stays at the altitude we let go
        target_p->altitude = state_p->altitude_m;
      }
    }
    else
    {
      // I added some throttle curve
      target_p->throttle += pow((gamepad_p->analog_LY / 1000.0f), 3) * 1.25f;
    
      if (target_p->throttle > MAX_TARGET_THROTTLE) target_p->throttle = MAX_TARGET_THROTTLE;
      else if (target_p->throttle < IDLE_THROTTLE) target_p->throttle = IDLE_THROTTLE;
    }
  }

}

void inner_control_loop()
{
  //↓↓↓↓↓↓↓↓↓↓   CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
  pid.errPitch = target_p->pitch_dps - state_p->pitch_dps;
  pid.errRoll = target_p->roll_dps - state_p->roll_dps;
  pid.errYaw = target_p->yaw_dps - state_p->yaw_dps;
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


  if (target_p->throttle > 200)
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
  //↑↑↑↑↑↑↑↑↑↑   PITCH D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

  //↓↓↓↓↓↓↓↓↓↓   ROLL D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
  pid.rollDout = -config_p->roll_d * (state_p->roll_dps - pid.rollDegsPrev);
  //↑↑↑↑↑↑↑↑↑↑   ROLL D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

  //↓↓↓↓↓↓↓↓↓↓   PITCH PID OUT   ↓↓↓↓↓↓↓↓↓↓
  pid.pitchPIDout = pid.pitchPout + pid.pitchIout + pid.pitchDout;
  if (pid.pitchPIDout > maxPID) pid.pitchPIDout = maxPID;
  else if (pid.pitchPIDout < -maxPID) pid.pitchPIDout = -maxPID;
  //↑↑↑↑↑↑↑↑↑↑   PITCH PID OUT   ↑↑↑↑↑↑↑↑↑↑

  //↓↓↓↓↓↓↓↓↓↓   ROLL PID OUT   ↓↓↓↓↓↓↓↓↓↓
  pid.rollPIDout = pid.rollPout + pid.rollIout + pid.rollDout;
  if (pid.rollPIDout > maxPID) pid.rollPIDout = maxPID;
  else if (pid.rollPIDout < -maxPID) pid.rollPIDout = -maxPID;
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
  
  if (flight_p->alt_hold_status == 1)
  {
    static uint8_t counter_alt_controller = 0;
    counter_alt_controller++;
    if (counter_alt_controller >= 10) // 50 Hz
    {
      counter_alt_controller = 0;


      //↓↓↓↓↓↓↓↓↓↓  CALCULATE CURRENT ERROR   ↓↓↓↓↓↓↓↓↓↓
      pid.errVel_z = target_p->velocity_z_ms - state_p->vel_up_ms;
      //↑↑↑↑↑↑↑↑↑↑   CALCULATE CURRENT ERROR   ↑↑↑↑↑↑↑↑↑↑

      //↓↓↓↓↓↓↓↓↓↓  ALTITUDE P CALCULATION  ↓↓↓↓↓↓↓↓↓↓
      pid.altPout = pid.errVel_z * config_p->altitude_p;
      //↑↑↑↑↑↑↑↑↑↑  ALTITUDE P CALCULATION   ↑↑↑↑↑↑↑↑↑↑

      //↓↓↓↓↓↓↓↓↓↓  ALTITUDE I CALCULATION   ↓↓↓↓↓↓↓↓↓↓
      if (flight_p->takeoff_status == 0)
      {
        pid.altIout += config_p->altitude_i * 0.01f * (pid.errVel_z + pid.errVel_z_prev);  //  0.01 = 0.5 * sampleTime
        if (pid.altIout > MAX_TARGET_THROTTLE) pid.altIout = MAX_TARGET_THROTTLE;
        else if (pid.altIout < IDLE_THROTTLE) pid.altIout = IDLE_THROTTLE;
      }
      //↑↑↑↑↑↑↑↑↑↑   ALTITUDE I CALCULATION   ↑↑↑↑↑↑↑↑↑↑

      //↓↓↓↓↓↓↓↓↓↓   ALTITUDE D CALCULATION   ↓↓↓↓↓↓↓↓↓↓
      //pid.altDout = -config.altitude_d * (states.velocity_z_ms - pid.velocity_z_ms_prev);
      pid.altDout = -config_p->altitude_d * (state_p->acc_up_ms2 + pid.acc_z_prev);
      pid.acc_z_prev = state_p->acc_up_ms2;
      //↑↑↑↑↑↑↑↑↑↑   ALTITUDE D CALCULATION   ↑↑↑↑↑↑↑↑↑↑

      //Altitude_D_Term_Filter();

      //↓↓↓↓↓↓↓↓↓↓   ALTITUDE PID OUT   ↓↓↓↓↓↓↓↓↓↓
      target_p->throttle = pid.altPout + pid.altIout + pid.altDout;
      if (target_p->throttle > MAX_TARGET_THROTTLE) target_p->throttle = MAX_TARGET_THROTTLE;
      else if (target_p->throttle < IDLE_THROTTLE) target_p->throttle = IDLE_THROTTLE;
      //↑↑↑↑↑↑↑↑↑↑   ALTITUDE PID OUT   ↑↑↑↑↑↑↑↑↑↑

      //↓↓↓↓↓↓↓↓↓↓   HOLD LAST ERROR FOR NEXT CALCULATION   ↓↓↓↓↓↓↓↓↓↓
      pid.errVel_z_prev = pid.errVel_z;
      pid.velocity_z_ms_prev = state_p->vel_up_ms;
      //↑↑↑↑↑↑↑↑↑↑   HOLD LAST ERROR FOR NEXT CALCULATION   ↑↑↑↑↑↑↑↑↑↑
    }
  }



  float comp_target_thr = target_p->throttle;
  // polinomial is specific to this drone
  // collect some data while hovering 
  // substract initial throttle value from all data it should start from 0
  // remove begining and end of the data
  // katsayilar = polyfit(batt_v, thr_zero, 2); this is the function for matlab (2 is for second order)
  float battery_compans_throttle = (-60.9742f * (telemetry_p->battery_voltage * telemetry_p->battery_voltage)) + (195.5156f * telemetry_p->battery_voltage) + 59.9253f;
  if (battery_compans_throttle < 0.0f) battery_compans_throttle = 0.0f;
  comp_target_thr += battery_compans_throttle;



  float cosAngAbs = cosf(fabs(state_p->pitch_deg) * DEG_TO_RAD) * cosf(fabs(state_p->roll_deg) * DEG_TO_RAD);
  if (cosAngAbs != 0) comp_target_thr += ((1.0f / cosAngAbs) - 1.0f) * comp_target_thr;




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
  float yaw_jump_reduce_thr = fabs(state_p->yaw_dps) * config_p->mag_declination_deg;
  thr_m1 -= yaw_jump_reduce_thr;
  thr_m2 -= yaw_jump_reduce_thr;
  thr_m3 -= yaw_jump_reduce_thr;
  thr_m4 -= yaw_jump_reduce_thr;

  if (thr_m1 < thr_min) thr_m1 = thr_min;
  if (thr_m2 < thr_min) thr_m2 = thr_min;
  if (thr_m3 < thr_min) thr_m3 = thr_min;
  if (thr_m4 < thr_min) thr_m4 = thr_min;

  set_throttle(thr_m1, thr_m2, thr_m3, thr_m4);
  //↑↑↑↑↑↑↑↑↑↑   OUTPUT TO THE MOTORS   ↑↑↑↑↑↑↑↑↑↑












}