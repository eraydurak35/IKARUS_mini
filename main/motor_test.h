#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "typedefs.h"

uint8_t motor_test(imu_t *imu, uint8_t motor_number);
float *get_motor_test_results();

#endif