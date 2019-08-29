#ifndef __SENSOR_H
#define __SENSOR_H
#include "elmo.h"
#include "moveBase.h"

#define CAMERA_DISTANCE	(31.5f)
#define DATA_LENGTH	(40)

typedef union{
	uint8_t data[16];
	float  value[4];
}coordinateGet_;

typedef struct{
	float distance_C[2];
	float distanceRecord_C[2];
	float distanceNow[2];
	float compensate[2];
	float recordYaw;
	float recordYawLast;
	float time;
	uint8_t status;
	uint8_t statusLast;
	uint8_t goFlag;
	float angle;
}visual_;


void USART2_IRQHandler(void);
void CheckVisionCommunication(uint8_t gaitCnt_t,float yaw,uint16_t timeRecord_t);
void SendTestCmd(float yaw);
void SendStopReady(void);
void GetJointPos(void);
void GetJointSpeed(void);
void GetGyroMessage(void);
uint8_t JudgeInformationBeyondVenue(void);
void JudgeInformation(float compensateX,float compensateY);
#endif



