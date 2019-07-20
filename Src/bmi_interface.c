#include "bmi_interface.h"
#include "bmi088.h"
#include "spi.h"
#include "uc_memory.h"

struct bmi08x_sensor_data raw_gyro_bmi088;
struct bmi08x_sensor_data raw_accel_bmi088;

int8_t user_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);
int8_t user_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len);
void delayUS(uint32_t us);

struct bmi08x_dev dev = {
        .accel_id = 7,
        .gyro_id = 6,
        .intf = BMI08X_SPI_INTF,  
        .read = user_spi_read,  
        .write = user_spi_write,  
        .delay_ms = HAL_Delay
};
  
  
int8_t user_spi_write(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  uint16_t pin = 0x0001;
  pin = pin << cs_pin;
  HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET );      // pull down cs bit
  delayUS(1);
  HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 5);
  HAL_SPI_Transmit(&hspi2, data, len, 5);
  delayUS(1);
  HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET ); // end of transfe
  return BMI08X_OK;
}

int8_t user_spi_read(uint8_t cs_pin, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  uint16_t pin = 0x0001;
  pin = pin << cs_pin;
  HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_RESET );      // pull down cs bit
  delayUS(1);
  HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 5);
  HAL_SPI_Receive(&hspi2, data, len, 5);
  delayUS(1);
  HAL_GPIO_WritePin(GPIOA, pin, GPIO_PIN_SET ); // end of transfe
  return BMI08X_OK;
}
  

void bmi_initialize(void)
{
  int8_t rslt;
  uint8_t data;

  /* Initializing the bmi085 sensors the below function will Initialize both accel and gyro sensors*/
  rslt = bmi088_init(&dev);
  
  if(rslt == BMI08X_OK) 
  {
    /* Assign the desired configurations */
    dev.accel_cfg.bw = BMI08X_ACCEL_BW_NORMAL;
    dev.accel_cfg.odr = BMI08X_ACCEL_ODR_50_HZ;
    dev.accel_cfg.range = BMI088_ACCEL_RANGE_3G;
    dev.accel_cfg.power = BMI08X_ACCEL_PM_ACTIVE;

    rslt = bmi08a_set_power_mode(&dev);
    /* Wait for 10ms to switch between the power modes - delay taken care inside the function*/

    rslt = bmi08a_set_meas_conf(&dev);  
    
  }
  
  if(rslt == BMI08X_OK) 
  {
    /* Read gyro chip id */
    rslt = bmi08g_get_regs(BMI08X_GYRO_CHIP_ID_REG, &data, 1, &dev);
    
    dev.gyro_cfg.power = BMI08X_GYRO_PM_NORMAL;

    rslt = bmi08g_set_power_mode(&dev);
    /* Wait for 30ms to switch between the power modes - delay taken care inside the function*/
      
    /* Assign the desired configurations */
    dev.gyro_cfg.odr = BMI08X_GYRO_BW_116_ODR_1000_HZ;
    dev.gyro_cfg.range = BMI08X_GYRO_RANGE_1000_DPS;
    dev.gyro_cfg.bw = BMI08X_GYRO_BW_116_ODR_1000_HZ;

    rslt = bmi08g_set_meas_conf(&dev);
  }
    
}
#ifdef BMI088 
void calibrate_imu(int n)
{
    float offset_gyro_x = 0 ,offset_gyro_y = 0, offset_gyro_z = 0;
    imu_data.offset.gyro_x = 0;
    imu_data.offset.gyro_y = 0;
    imu_data.offset.gyro_z = 0;

    for(int i = 0; i< n; i++)
    {
        HAL_Delay(1);
        if( i%30 == 0 ){
            HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
        }
        get_imu_data(&imu_data);
        offset_gyro_x += imu_data.raw.gyro_x;
        offset_gyro_y += imu_data.raw.gyro_y;
        offset_gyro_z += imu_data.raw.gyro_z;
    }
    imu_data.offset.gyro_x = offset_gyro_x/n;
    imu_data.offset.gyro_y = offset_gyro_y/n;
    imu_data.offset.gyro_z = offset_gyro_z/n;

    FEE_WriteDataFloat(0x00, imu_data.offset.gyro_x);
    FEE_WriteDataFloat(0x04, imu_data.offset.gyro_y);
    FEE_WriteDataFloat(0x08, imu_data.offset.gyro_z);

}

void get_imu_data(struct imu_data_t* data)
{
    int8_t rslt;
   
    /* Read the sensor data into the sensor data instance */
    rslt = bmi08g_get_data(&raw_gyro_bmi088, &dev);
  
    /* Read the sensor data into the sensor data instance */
    rslt = bmi08a_get_data(&raw_accel_bmi088, &dev);
  
    // convert data in standard unit
    // accelerometer 3g
    // gyroscope 1000deg/s
    data->acc_x = ((float)raw_accel_bmi088.x * 3 )/32768;
    data->acc_y = ((float)raw_accel_bmi088.y * 3 )/32768;
    data->acc_z = ((float)raw_accel_bmi088.z * 3 )/32768;
    data->gyro_x = ((float)raw_gyro_bmi088.x  -  data->offset.gyro_x)* 1000 /32768/57.29578;
    data->gyro_y = ((float)raw_gyro_bmi088.y  -  data->offset.gyro_y)* 1000 /32768/57.29578;
    data->gyro_z = ((float)raw_gyro_bmi088.z  -  data->offset.gyro_z)* 1000 /32768/57.29578;
}

float get_temperature(void)
{
  int32_t tmp;
  bmi08a_get_sensor_temperature(&dev, &tmp);
  return ((float)tmp)/1000;
}

#endif



void delayUS(uint32_t us) {
	volatile uint32_t counter = 7*us;
	while(counter--);
}
