#include "thermal_control.h"
#include "FreeRTOS.h"
#include "task.h"

BaseType_t result = pdFALSE;
TaskHandle_t thermal_crtl_h;

float temperature_f = 25.0;
float thermal_Kp = 3, thermal_Ki = 0.2;

void thermal_ctrl_task(void *p);


void limit_int(int *x , int max, int min)
{
  if(*x > max)
    *x = max;
  if(*x < min)
    *x = min;
}


void thermal_ctrl_init(void)
{
  result = xTaskCreate(thermal_ctrl_task,
              "thermal_ctrl",
              64,
              (void*)1 ,
               10,
              &thermal_crtl_h
              );
  if(result ==! pdPASS)
  {
      vTaskDelete(thermal_crtl_h);
  }
}


void thermal_ctrl_task(void *p)
{
  float Intergral = 0, err, out;
  int pwm = 200;
  for(;;)
  {
    temperature_f = get_temperature();
    err = temperature_f - stable_temperature;
    out = err * thermal_Kp;
    /*Intergral += err;
    if(Intergral > 50)
      Intergral = 50;
    if(Intergral < -50)
      Intergral = -50;
    out += Intergral;*/
    pwm -= out;
    limit_int( &pwm, 999, 0);
    TIM4->CCR3 = pwm;
    vTaskDelay(update_period);
  }
}

