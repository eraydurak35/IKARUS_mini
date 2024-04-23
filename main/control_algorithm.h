#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include "comminication.h"
#include "state_estimator.h"

#define MAX_TARGET_THROTTLE 600
#define IDLE_THROTTLE 150

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
  uint8_t arm_status;
  uint8_t alt_hold_status;
  uint8_t pos_hold_status;
  uint8_t takeoff_status;
} flight_t;

void control_init(gamepad_t *gamepad, telemetry_t *telemetry, flight_t *flight, target_t *target, states_t *state, config_t *config);
void check_flight_mode();
void arm();
void disarm();
void flight_control();
void outer_control_loop();
void inner_control_loop();

#endif