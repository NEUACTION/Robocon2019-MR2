#ifndef VLOOP_
#define VLOOP_

#include "moveBase.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "define.h"
#include "can.h"
#include "elmo.h"
#define N	(200)
#define HAA 1
#define HFE 2
#define KFE 3
#define VEL_MAX_3508		(20.0f)


typedef struct
{
	float hip;
	float knee;
}legJoint;

typedef struct
{	
	int32_t can_status;
	
	int32_t canId;
	
}CommandType;

typedef struct
{
	legJoint speed;
	
	float desiredVel[3];
	
	float velErr;
	
	float acc;
	
	float dec;
	
	float kp;
	
	float ki;
	
	float iOut;
	
	float output;
	
	float maxOutput;
	
	legJoint sendSpeed;
	
}VelCtrlType;

typedef struct
{
	legJoint actualPos;
	
	float desiredPos;
	
	float posErr,posErrLast;
	
	float posVel,acc;
	
	float basicPos;
	
	float kp;
	
	float kd;
	
	float output;
	
}PosCtrlType;

typedef struct
{	
	int desiredPos[3][20];
	
	float desiredTime;

	uint8_t motorNum;
	
	legJoint output;
	
	legJoint referSpeed;
	
	legJoint velOutput;
	
	float posOutput;
	
	float velLimit;
	
	float pulseMaxLimit;
	
	float pulseMinLimit;
	
	float kp;
	
	float kd;
	
	float ki;
	
	legJoint lastPos;
	
	legJoint newPos;
	
	legJoint NextPos;
	
	
	uint8_t index;
	
	
}PTCtrlType;


typedef struct
{
	uint32_t unitMode;
  
  int32_t time;
	
  FunctionalState status;
  
	float output;
	
	int encoder5012B;
	
	int target5012B;
	
	VelCtrlType velCtrl;
	
	PosCtrlType posCtrl;
	
	CommandType command;
	
	PTCtrlType ptCtrl;
	
}MotorType;

legJoint transfer(uint8_t canNum,float pointAngleHip,float pointAngleknee);
void VelOutput(float pointAngleHip,float pointAngleNextHip,float pointAngleKnee,float pointAngleNextKnee,uint8_t canNum);//canNum=1~4;
float MaxMinLimit(float val,float limit);
legJoint PTCtrlTransfer(uint8_t motorNum);
legJoint PointInput(uint8_t motorNum,float pointAngleHip,float pointAngleNextHip,float pointAngleKnee,float pointAngleNextKnee);
float HipTrans(int32_t pulse);
float KneeTrans(int32_t pulse);
#endif 










