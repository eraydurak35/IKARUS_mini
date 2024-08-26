#include "motor_test.h"
#include "typedefs.h"
#include "gpio.h"
#include "math.h"

static uint16_t counter = 0;
static float result[4];
static float base_noise_level = 0;

uint8_t motor_test(imu_t *data, uint8_t motor_number)
{

    if (counter < 100)
    {
        base_noise_level += sqrtf(data->accel_ms2[X] * data->accel_ms2[X] + data->accel_ms2[Y] * data->accel_ms2[Y] + data->accel_ms2[Z] * data->accel_ms2[Z]);
    }
    else if (counter == 100)
    {
        base_noise_level /= 100.0f;

        if (motor_number == 1)
            set_throttle(200, 0, 0, 0);
        else if (motor_number == 2)
            set_throttle(0, 200, 0, 0);
        else if (motor_number == 3)
            set_throttle(0, 0, 200, 0);
        else if (motor_number == 4)
            set_throttle(0, 0, 0, 200);

        result[motor_number - 1] = 0;
    }
    else if (counter > 1600 && counter <= 6600)
    {
        float acc_vector = sqrtf(data->accel_ms2[X] * data->accel_ms2[X] + data->accel_ms2[Y] * data->accel_ms2[Y] + data->accel_ms2[Z] * data->accel_ms2[Z]) - base_noise_level;
        result[motor_number - 1] += acc_vector * acc_vector;
    }
    else if (counter > 6600)
    {
        set_throttle(0, 0, 0, 0);
        result[motor_number - 1] = sqrtf((result[motor_number - 1] / 5000.0f));
        counter = 0;
        base_noise_level = 0;
        return 0;
    }

    counter++;
    return 1;

}

float *get_motor_test_results()
{
    return result;
}