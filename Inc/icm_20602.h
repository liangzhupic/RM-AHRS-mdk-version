#ifndef  __ICM_20602_H
#define __ICM_20602_H


#include "FreeRTOS.h"
#include "spi.h"
#include "stm32f1xx_hal.h"

// interal register address of icm20602
#define      CONFIG                     0x1A
#define      GYRO_CONFIG        0x1B
#define      ACCEL_CONFIG       0x1C
#define     ACCEL_CONFIG2       0x1D
#define     LP_MODE_CFG         0x1F
#define     FIFO_EN                         0x23
#define     INT_ENABLE                  0x38
#define     INT_PIN_CFG                 0x37
#define     ACCEL_XOUT_H            0x3B
#define     TEMP_OUT_H               0x41
#define     GYRO_XOUT_H             0x43
#define     USER_CTRL                   0x6A
#define     PWR_MGMT_1              0x6B
#define     PWR_MGMT_2              0x6C
#define     I2C_IF                              0x70
#define     WHO_AM_I                    0x75

// write / read bit
#define     add_read_bit(adr)       adr = adr|0x80
#define     add_write_bit(adr)      adr

struct imu_data_t
{
    struct raw_t
    {
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
        int16_t gyro_x;
        int16_t gyro_y;
        int16_t gyro_z;
    }raw;
    struct offset_t
    {
        float gyro_x;
        float gyro_y;
        float gyro_z;
    }offset;


    float   acc_x;
    float   acc_y;
    float   acc_z;
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;
};

extern uint16_t calibration_time;
extern struct imu_data_t   imu_data;

extern   void    write_icm_data(uint8_t adr, uint8_t data);
extern   void    read_icm_data(uint8_t adr, uint8_t size, uint8_t* data);
extern   void   initlize_icm(void);
extern   void   get_imu_data(struct imu_data_t* data);
extern   void 	calibrate_imu(int n);



#endif
