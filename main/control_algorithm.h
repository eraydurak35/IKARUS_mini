#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h>
#include "comminication.h"
#include "state_estimator.h"
#include "typedefs.h"

#define MAX_TARGET_THROTTLE 600
#define IDLE_THROTTLE 100



void control_init(gamepad_t *gamepad, telemetry_small_t *telemetry, flight_t *flight, target_t *target, states_t *state, config_t *config);
void check_flight_mode();
void flight_control();

#endif