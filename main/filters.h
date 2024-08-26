#ifndef FILTERS_H
#define FILTERS_H

#include <stdio.h>
#include "typedefs.h"

typedef struct
{
    uint8_t size;
    float buffer[7];
    uint8_t index;
} fir_filter_t;

typedef struct 
{
    float sample_rate;
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float x1;
    float x2;
    float y1;
    float y2;
} biquad_lpf_t;

typedef struct
{
    float sample_rate;
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
    float x1;
    float x2;
    float y1;
    float y2;
} notch_filter_t;

typedef struct {
    uint8_t size;
    float buffer[10];
    float sorted_buffer[10];
} median_filter_t;

float median_filter(median_filter_t *filter, float new_data);

void fir_filter_init(fir_filter_t *fir);
void fir_filter(fir_filter_t *fir, float *value);
void fir_filter_custom_gain(fir_filter_t *fir, const float *gain, float *value);
void apply_fir_filter_to_imu(imu_t *imu, fir_filter_t *fir);

void notch_filter_init(notch_filter_t *notch);
void notch_configure(float cf, float bw, notch_filter_t *notch);
void notch_filter(notch_filter_t *notch, float *value);
void apply_notch_filter_to_imu(imu_t *imu, notch_filter_t *notch);

void biquad_lpf_configure(float cf, biquad_lpf_t *lowpass);
void biquad_lpf(biquad_lpf_t *lowpass, float *value);
void biquad_lpf_array_init(biquad_lpf_t *lpf);
void apply_biquad_lpf_to_imu(imu_t *imu, biquad_lpf_t *lpf);
#endif