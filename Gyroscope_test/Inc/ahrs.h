#ifndef _ahrs_h
#define _ahrs_h

#include "icm_20602.h"
#include "task.h"
#include "math.h"
#include "semphr.h"
#include "FreeRTOS.h"


//----------------------------------------------------------------------------------------------------
// Variable declaration
extern uint8_t Can_ID ;

extern int instability_fix;
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

typedef struct quaternion
{
    float q0;
    float q1;
    float q2;
    float q3;
}quaternion;
extern quaternion NormoliseQuaternion;

struct euler
{
    volatile float pitch;
   volatile float roll;
   volatile float yaw;
};

struct ref_g_t{
    float x;
    float y;
    float z;
};
extern uint8_t mode;
extern struct euler EulerAngle;
extern float ahrs_count, ahrs_count_sec;
extern uint8_t change_recipNorm;

extern struct ref_g_t ref_g;
//task

extern BaseType_t AHRS_t;
extern TaskHandle_t AHRS_h;

extern SemaphoreHandle_t ahrs_sem;
//---------------------------------------------------------------------------------------------------
// Function declarations

void AHRSCalulation(struct imu_data_t , quaternion*);
void IMUCalulation(struct imu_data_t , quaternion*);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,quaternion *);

void AhrsTaskCreate(void);
void AhrsTask(void*);



#endif
