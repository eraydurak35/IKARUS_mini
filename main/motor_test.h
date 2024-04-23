#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "icm42688p.h"

uint8_t motor_test(icm42688p_t *imu, uint8_t motor_number);
float *get_motor_test_results();

#endif