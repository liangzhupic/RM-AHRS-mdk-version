#include "thermal_control.h"
#include "FreeRTOS.h"
#include "task.h"

BaseType_t result = pdFALSE;
TaskHandle_t thermal_crtl_h;

float temperature_f = 25.0;
float thermal_Kp = 30, thermal_Ki = 2;
float Intergral = 0, err, out;

//int stable_temperature = 45;

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
  
  int pwm = 200;
  for(;;)
  {
    temperature_f = get_temperature();
    err = temperature_f - stable_temperature;
    out = err * thermal_Kp;
    Intergral += err * thermal_Ki * (update_period/1000);
    if(Intergral > 500)
      Intergral = 500;
    if(Intergral < -500)
      Intergral = -500;
    out += Intergral;
    pwm = -out;
    limit_int( &pwm, 999, 0);
    TIM4->CCR3 = pwm;
    if(fabs(err) > 1)
    {
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
      vTaskDelay(update_period);
    }
    else{
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
      vTaskDelay(update_period);
    }
    
  }
}

