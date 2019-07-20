# RM-AHRS-mdk-version

## usage
* uncomment code block of CAN transmission in ahrs.c if using in RM i.e.
```c
//CAN1_Send_Msg(Can_ID+1, (int16_t)(imu_data.gyro_x*1000), (int16_t)(imu_data.gyro_z*1000) , \
//(int16_t)(EulerAngle.pitch*5729.5), (int16_t)(EulerAngle.yaw*5729.5), 4);
```

* add bmi088 sensor in the latest release
* use bmi088 sensor by default. you can change sensor written in code to match you board's sensor by changing the MACRO in ahrs.c 
e.g. 
```c 
define BMI088 1 
// define ICM20602 1
```
