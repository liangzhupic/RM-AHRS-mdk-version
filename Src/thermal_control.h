#ifndef __thermal_ctrl_H__
#define __thermal_ctrl_H__

#include "tim.h"
#include "bmi_interface.h"

#define enable_thermal_control 1
#define update_period 1280

#define stable_temperature 48

extern float temperature_f;

#endif

