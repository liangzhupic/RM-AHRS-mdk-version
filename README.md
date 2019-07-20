# RM-AHRS-mdk-version

## Usage
* Uncomment code block of CAN transmission in ahrs.c if using in RM i.e.
```c
//CAN1_Send_Msg(Can_ID+1, (int16_t)(imu_data.gyro_x*1000), (int16_t)(imu_data.gyro_z*1000) , \
//(int16_t)(EulerAngle.pitch*5729.5), (int16_t)(EulerAngle.yaw*5729.5), 4);
```

* Add bmi088 sensor in the latest release
* Use bmi088 sensor by default. You can change sensor written in code to match you board's sensor by changing the MACRO in ahrs.c 
e.g. 
```c 
 #define BMI088 1 
// #define ICM20602 1
```
* Add a thermal control feature. There're only a few parameters used to tune.
```c
 #define enable_thermal_control 1
 #define update_period 1280
 #define stable_temperature 48
```  
* Red LED (PB11) will blink when thermal control part under-temerature or over-temperature. Plz calibrate the sensor offset when LED off (means within +-0.75 centigrade ).