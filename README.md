# RM-AHRS-mdk-version

## usage
* uncomment code block of CAN transmission if using in RM
* add bmi088 sensor in the latest release
* use bmi088 sensor by default. you can change sensor written in code to match you board's sensor by changing the MACRO in ahrs.c 
e.g. 
```c 
define BMI088 1 
// define ICM20602 1
```
