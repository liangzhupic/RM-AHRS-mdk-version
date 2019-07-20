#include "thermal_control.h"
#include "FreeRTOS.h"
#include "task.h"

BaseType_t result = pdFALSE;
TaskHandle_t thermal_crtl_h;

float temperature_f = 25.0;

void thermal_ctrl_task(void *p);

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
#define update_period 1250
  for(;;)
  {
    temperature_f = get_temperature();
    vTaskDelay(update_period);
  }
}
