#ifndef __BMI_INTERFACE_H__
#define __BMI_INTERFACE_H__

#include  "icm_20602.h"
#include "ahrs.h"

#define update_period 1280
#define stable_temperature 48

float get_temperature(void);

#endif