#include <stdio.h>
#include "filters.h"
#include "icm42688p.h"
#include "math.h"


static const float fir_gain[7] = 
{
  0.142386908288827163f,
  0.142856919905681196f,
  0.143139372760745009f,
  0.143233598089493264f,
  0.143139372760745009f,
  0.142856919905681196f,
  0.142386908288827163f
};


void fir_filter_init(fir_filter_t *fir)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        fir[i].size = 7;
        fir[i].index = 0;
    }
}


void fir_filter(fir_filter_t *fir, float *value)
{
    fir->buffer[fir->index] = *value;
    fir->index++;

    if (fir->index == fir->size) fir->index = 0;

    float temp = 0;
    uint8_t sumIndex = fir->index;

    for (uint8_t n = 0; n < fir->size; n++)
    {
    if (sumIndex > 0) sumIndex--;
    else sumIndex = fir->size - 1;
    temp += fir_gain[n] * fir->buffer[sumIndex];
    }
    *value = temp;
}



void fir_filter_custom_gain(fir_filter_t *fir, const float *gain, float *value)
{
    fir->buffer[fir->index] = *value;
    fir->index++;

    if (fir->index == fir->size) fir->index = 0;

    float temp = 0;
    uint8_t sumIndex = fir->index;

    for (uint8_t n = 0; n < fir->size; n++)
    {
    if (sumIndex > 0) sumIndex--;
    else sumIndex = fir->size - 1;
    temp += gain[n] * fir->buffer[sumIndex];
    }
    *value = temp;
}


void apply_fir_filter_to_imu(icm42688p_t *imu, fir_filter_t *fir)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        fir_filter(&fir[i], &imu->gyro_dps[i]);
        fir_filter(&fir[i+3], &imu->accel_ms2[i]);
    }
}




void notch_filter_init(notch_filter_t *notch)
{
    for (uint8_t i = 0; i < 18; i++)
    {
        notch[i].sample_rate = 2000.0f;
        notch_configure(200.0f, 25.0f, &notch[i]);
    }
    
}


//Q = notchCenterFreq_hz / bandwidth_hz
void notch_configure(float cf, float bw, notch_filter_t *notch)
{
    float Q = cf / bw;
    float omega = 2.0 * M_PI * cf / notch->sample_rate;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0 * Q);

    notch->b0 = 1;
    notch->b1 = -2 * cs;
    notch->b2 = 1;
    notch->a0 = 1 + alpha;
    notch->a1 = -2 * cs;
    notch->a2 = 1 - alpha;

    // prescale flter constants
    notch->b0 /= notch->a0;
    notch->b1 /= notch->a0;
    notch->b2 /= notch->a0;
    notch->a1 /= notch->a0;
    notch->a2 /= notch->a0;
}

// perform one filtering step
void notch_filter(notch_filter_t *notch, float *value)
{
    static float x = 0;
    static float y = 0;
    x = *value;
    y = notch->b0 * x + notch->b1 * notch->x1 + notch->b2 * notch->x2 - notch->a1 * notch->y1 - notch->a2 * notch->y2;
    notch->x2 = notch->x1;
    notch->x1 = x;
    notch->y2 = notch->y1;
    notch->y1 = y;
    *value = y;
}




void apply_notch_filter_to_imu(icm42688p_t *imu, notch_filter_t *notch)
{
    for (uint8_t i = 0; i < 3; ++i) 
    {
        for (uint8_t j = 0; j < 3; ++j) 
        {
            notch_filter(&notch[i * 3 + j], &imu->gyro_dps[i]);
            notch_filter(&notch[9 + i * 3 + j], &imu->accel_ms2[i]);
        }
    }

}

void biquad_lpf_configure(float cf, biquad_lpf_t *lowpass)
{
    float omega = 2.0f * M_PI * cf / 1000.0f;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / 2.0f;

    lowpass->b0 = (1.0f - cs) / 2.0f;
    lowpass->b1 = 1.0f - cs;
    lowpass->b2 = (1.0f - cs) / 2.0f;
    lowpass->a0 = 1.0f + alpha;
    lowpass->a1 = -2.0f * cs;
    lowpass->a2 = 1.0f - alpha;

    lowpass->b0 /= lowpass->a0;
    lowpass->b1 /= lowpass->a0;
    lowpass->b2 /= lowpass->a0;
    lowpass->a1 /= lowpass->a0;
    lowpass->a2 /= lowpass->a0;
}

void biquad_lpf(biquad_lpf_t *lowpass, float *value)
{
    static float x = 0;
    static float y = 0;
    x = *value;
    y = lowpass->b0 * x + lowpass->b1 * lowpass->x1 + lowpass->b2 * lowpass->x2 - lowpass->a1 * lowpass->y1 - lowpass->a2 * lowpass->y2;
    lowpass->x2 = lowpass->x1;
    lowpass->x1 = x;
    lowpass->y2 = lowpass->y1;
    lowpass->y1 = y;
    *value = y;
}

void biquad_lpf_array_init(biquad_lpf_t *lpf)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        biquad_lpf_configure(50.0f, lpf + i);
    }
}

void apply_biquad_lpf_to_imu(icm42688p_t *imu, biquad_lpf_t *lpf)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        biquad_lpf(&lpf[i], &imu->gyro_dps[i]);
        biquad_lpf(&lpf[i+3], &imu->accel_ms2[i]);
    }
}