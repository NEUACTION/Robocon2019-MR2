#ifndef __PID_H
#define __PID_H

#include "moveBase.h"

typedef struct{
	
	float kp;
	float ki;
	float kd;
	float integral;
	float errRecord;
	float integralLimit;
}pid_t;

typedef struct{
	float k;
	float b;
	float limit;
}gyro_t;

typedef struct
{
    float angleErr;
    float wErr;
    float targetAngle;
    float targetW;
    float outsideKp;
    float insideKp;
    float angleLoopOut;
    float gainK;
    float wLoopIn;
    float outPut;
    float max1;
    float max2;
}Serial_;

void PIDParaInit(void);
float DistanceControl(float posErr);
float YawControl(float actulYaw,float desireYaw);
int16_t timeControl(float desireX,float desireY);
float LF_RBAngleControl(float dAngle,float actulAngle,float actulGyro);
float RF_LBAngleControl(float dAngle,float actulAngle,float actulGyro);
void sreialPIDInit(void);
#endif


