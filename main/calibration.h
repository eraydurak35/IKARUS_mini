#ifndef CALIBRATION_H
#define CALIBRATIO_H

#include "icm42688p.h"
#include "qmc5883l.h"
#include "typedefs.h"

uint8_t gyro_calibration(imu_t *imu);
uint8_t accelerometer_calibration(imu_t *imu);
uint8_t magnetometer_calibration(magnetometer_t *mag);
void reset_calibration_data(calibration_t *data);





#endif