#ifndef STORAGE_H
#define STORAGE_H

#include "comminication.h"

void read_config(config_t *cfg);
void save_config(config_t *cfg);
void save_mag_cal(float *mg_cal);
void read_mag_cal(float *mg_cal);


#endif