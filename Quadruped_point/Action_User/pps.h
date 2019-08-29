#ifndef __PPS_H
#define __PPS_H
#include "stdint.h"

#define GET_PPS_VALUE_NUM      6
/*接受几个来自定位系统的uint8_t数据*/ /* 6 * 4byte = 24*/
#define GET_PPS_DATA_NUM       24


typedef union{
	uint8_t data[GET_PPS_DATA_NUM];
	float  value[GET_PPS_VALUE_NUM];
}PosSend_t;

/*定位系统返回的值*/
typedef struct{
		/*定位系统返回的角度*/
		float ppsAngle ;
		/*定位系统返回的X值*/
		float ppsX ;
		/*定位系统返回的Y值*/
		float ppsY ;
		/*定位系统返回的X轴速度*/
		float ppsSpeedX;
		/*定位系统返回的Y轴速度*/
		float ppsSpeedY;
		/*定位系统的z轴角速度*/
		float ppsWZ ;
}Pos_t;


void TalkOpsToGetReady(void);
/*初始化并且让程序等待定位系统发数*/
void WaitOpsPrepare(void);

void SetOpsReady(uint8_t flag);
void SetAngle(float setValue);
void SetX(float setValue);
void SetY(float setValue);
void SetSpeedX(float setValue);
void SetSpeedY(float setValue);
void SetWZ(float setValue);
/*定位系统角度清零*/
void Correct(void);
/*定位系统准备完毕*/
uint8_t GetOpsReady(void);
/*返回定位系统的角度*/
float GetAngle(void);
/*返回定位系统的X值*/
float GetX(void);
/*返回定位系统的Y值*/
float GetY(void);
/*返回定位系统的X轴的速度*/
float GetSpeedX(void);
/*返回定位系统的角度*/
float GetSpeedY(void);
/*返回定位系统的Z轴角速度值*/
float GetWZ(void);


#endif 

