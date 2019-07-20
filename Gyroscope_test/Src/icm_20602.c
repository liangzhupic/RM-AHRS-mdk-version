#include  "icm_20602.h"
#include  "stdio.h"
#include "uc_memory.h"
#include "can.h"

void    write_icm_data(uint8_t adr, uint8_t data);
void    read_icm_data(uint8_t adr, uint8_t size, uint8_t* data);
void    initlize_icm(void);
void    get_imu_data(struct imu_data_t* data);
void    calibrate_imu(int n);
struct imu_data_t   imu_data;

uint16_t calibration_time= 5000;


void    calibrate_imu(int n)
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
void   get_imu_data(struct imu_data_t* data)
{
    uint8_t rx[10];
    read_icm_data(ACCEL_XOUT_H, 8, rx);
    data->raw.acc_x = rx[0]<<8 | rx[1];
    data->raw.acc_y = rx[2]<<8 | rx[3];
    data->raw.acc_z = rx[4]<<8 | rx[5];
    read_icm_data(GYRO_XOUT_H, 8, rx);
    data->raw.gyro_x = rx[0]<<8 | rx[1];
    data->raw.gyro_y = rx[2]<<8 | rx[3];
    data->raw.gyro_z = rx[4]<<8 | rx[5];
    // convert data in standard unit
    // accelerometer 2g
    // gyroscope 1000deg/s
    data->acc_x = ((float)data->raw.acc_x * 2 )/32768;
    data->acc_y = ((float)data->raw.acc_y * 2 )/32768;
    data->acc_z = ((float)data->raw.acc_z * 2 )/32768;
    data->gyro_x = ((float)data->raw.gyro_x  -  data->offset.gyro_x)* 1000 /32768/57.29578;
    data->gyro_y = ((float)data->raw.gyro_y  -  data->offset.gyro_y)* 1000 /32768/57.29578;
    data->gyro_z = ((float)data->raw.gyro_z  -  data->offset.gyro_z)* 1000 /32768/57.29578;
}

void    read_icm_data(uint8_t adr, uint8_t size, uint8_t* data)
{
    add_read_bit(adr);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET );      // pull down cs bit
    HAL_SPI_Transmit(&hspi2,  &adr, 1, 10);
    HAL_SPI_Receive(&hspi2,  data, size, 10 );
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET ); // end of transfer
}

void    write_icm_data(uint8_t adr, uint8_t data)
{
    uint8_t tx[]={
        add_write_bit(adr),
        data
    };
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET );      // pull down cs bit
    HAL_SPI_Transmit(&hspi2, tx  , 2, 1 );
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET ); // end of transfer
}

void    initlize_icm(void)
{
    write_icm_data(PWR_MGMT_1, 0x80);   // reset device
    HAL_Delay(2);
    write_icm_data(PWR_MGMT_1, 0x01);   // active device from sleep
    HAL_Delay(2);
    write_icm_data(CONFIG, 0x01); //177 hz noise low pass filter
    write_icm_data(GYRO_CONFIG, 0x10);  // max 1000deg/s
    write_icm_data(ACCEL_CONFIG, 0x00); //max 2g
    write_icm_data(ACCEL_CONFIG2,0x04); // set up acc noise filter of 15.5hz

    HAL_Delay(20);
    FEE_Init();

//    calibrate_imu(calibration_time);

    imu_data.offset.gyro_x = FEE_ReadDataFloat(0x00);
    imu_data.offset.gyro_y = FEE_ReadDataFloat(0x04);
    imu_data.offset.gyro_z = FEE_ReadDataFloat(0x08);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
}

