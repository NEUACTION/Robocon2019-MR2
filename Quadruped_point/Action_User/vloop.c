#include "vloop.h"
#include "math.h"
#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"
#include "module_RED.h"
#include "app_cfg.h"
#include "sensor.h"
#include "light.h"
#include "errorHand.h"
#include "stm32f4xx_it.h"
#include "init.h"
#include "dma.h"

/*肩关节和膝关节电机控制变量*/
MotorType motor[4];
extern int testTime;
extern uint8_t testFlag;
extern uint16_t timeRecord;
extern leg_ legOne, legTwo, legThr, legFor;  
extern int can1ErrNum[8];
extern uint16_t DebugUSARTSendBuffCnt2;
extern uint8_t DebugUSARTSendBuf2[DEBUG_USART_SEND_BUF_CAPACITY_2];
extern uint8_t DebugUSARTDMASendBuf2[DEBUG_USART_SEND_BUF_CAPACITY_2];
int record[8]={0};
void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	UnionDataType Msg1;
	uint8_t buffer[4]={0};
	uint8_t len=4;
	uint32_t StdId=0;
	
	for(uint8_t i=1;i<=4;i++)
	 motor[i-1].command.canId=i;
	
	CAN_RxMsg(CAN1,&StdId,buffer,&len);
	for(uint8_t i=0;i<=3;i++)
	Msg1.data8[i]=buffer[i];
	//读取12个电机位置和速度
	
	switch (StdId)
	{
			//VX   读取位置
			case 0x221:
				can1ErrNum[0]-=2;
				motor[0].posCtrl.actualPos.hip =Msg1.data32;
				
				break;
			case 0x222:
				can1ErrNum[1]-=2;
				motor[0].posCtrl.actualPos.knee =Msg1.data32;
				
				break;
			case 0x223:
				can1ErrNum[2]-=2;
				motor[1].posCtrl.actualPos.hip =Msg1.data32;
				
				break;
			case 0x224:
				can1ErrNum[3]-=2;
				motor[1].posCtrl.actualPos.knee =Msg1.data32;
				
				break;
			case 0x225:
				can1ErrNum[4]-=2;
				motor[2].posCtrl.actualPos.hip =Msg1.data32;
				
				break;
			case 0x226:
				can1ErrNum[5]-=2;
				motor[2].posCtrl.actualPos.knee =Msg1.data32;
				
				break;
			case 0x227:
				can1ErrNum[6]-=2;
				motor[3].posCtrl.actualPos.hip =Msg1.data32;
				break;
			case 0x228:
				can1ErrNum[7]-=2;
				motor[3].posCtrl.actualPos.knee =Msg1.data32;
				break;
			
		
			case 0x201:
				motor[0].velCtrl.speed.hip =Msg1.data32;
				break;
			case 0x202:
				motor[0].velCtrl.speed.knee =Msg1.data32;
				break;
			case 0x203:
				motor[1].velCtrl.speed.hip =Msg1.data32;
				break;
			case 0x204:
				motor[1].velCtrl.speed.knee =Msg1.data32;
				break;
			case 0x205:
				motor[2].velCtrl.speed.hip =Msg1.data32;
				break;
			case 0x206:
				motor[2].velCtrl.speed.knee =Msg1.data32;
				break;
			case 0x207:
				motor[3].velCtrl.speed.hip =Msg1.data32;
				break;
			case 0x208:
				motor[3].velCtrl.speed.knee =Msg1.data32;
				break;
		default:break;
	
	}	
	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

/**
* @brief  由关节转动角度转化为丝杠电机的转动
* @param  pointAngle为输入的点（均为正角度，弧度制），CanNum为点对应的电机编号（1,2,3...）
* @author ACTION
* @note
*/

legJoint transfer(uint8_t canNum,float pointAngleHip,float pointAngleknee)
{
    float length=0;
    float mechanicAngle=0;
    legJoint paulse={0};
    
	mechanicAngle =128.75f*PI/180.0f-pointAngleHip;//     -3.25 ~ 90    44.282
	length=sqrt(220.05f*220.05f+50.931f*50.931f-2.0f*50.931f*220.05f*cos(mechanicAngle));//209.75  252.19            187
	paulse.hip=-(length-183)/8.0f*8192.0f;

	mechanicAngle=145.11f*PI/180.0f-pointAngleknee;//   30.11~ 130    
	length = sqrt(231.355f*231.355f+50.0f*50.0f-2.0f*231.355f*50.0f*cos(mechanicAngle));//256.52                 185
	paulse.knee=-(length-183)/8.0f*8192.0f;
     
     return paulse;   
}



/**
* @brief  由位置信息转化成速度指令发送给转接板
* @param  pointAngle为输入的点（均为正角度，弧度制），CanNum为点对应的电机编号（1,2,3...）
* @author ACTION
* @note
*/
void VelOutput(float pointAngleHip,float pointAngleNextHip,float pointAngleKnee,float pointAngleNextKnee,uint8_t canNum)//canNum=1~4;
{
	//右边4 5 6，10 11 12号电机输出为正
	PointInput(canNum-1,pointAngleHip,pointAngleNextHip,pointAngleKnee,pointAngleNextKnee);
	//发出速度指令
	VelCrl(CAN1, canNum,motor[canNum-1].velCtrl.sendSpeed.hip*1000,motor[canNum-1].velCtrl.sendSpeed.knee*1000);
			
}
//限幅函数
float MaxMinLimit(float val,float limit)
{
	if(val > limit) val =  limit;
	if(val <-limit) val = -limit;
	
	return val;
}

float iout = 0;


/**
* @brief 	将位置转化成速度环PID
* @param    motorNum电机编号（0,1,2,3....11）
* @return   motor[motorNum].ptCtrl.output 速度输出
* @author 	ACTION
* @note kp 10ms 0.15,0.16
*/
extern uint8_t startOver;
extern  int correctCnt;

legJoint PTCtrlTransfer(uint8_t motorNum)
{
	legJoint returnSpeed={0};
	
	/*******肩关节控制*******/
	motor[motorNum].ptCtrl.desiredTime=timeRecord/10;
	
	motor[motorNum].ptCtrl.velOutput.hip = (motor[motorNum].ptCtrl.newPos.hip - motor[motorNum].posCtrl.actualPos.hip)/ (motor[motorNum].ptCtrl.desiredTime);			//做差值	
	motor[motorNum].ptCtrl.referSpeed.hip=(motor[motorNum].ptCtrl.NextPos.hip - motor[motorNum].ptCtrl.newPos.hip)/ (motor[motorNum].ptCtrl.desiredTime);
	
	motor[motorNum].ptCtrl.output.hip=MaxMinLimit((motor[motorNum].ptCtrl.velOutput.hip+motor[motorNum].ptCtrl.referSpeed.hip)/2.0f,motor[motorNum].ptCtrl.velLimit);
	returnSpeed.hip=motor[motorNum].ptCtrl.output.hip;
	
	/*******膝关节控制*******/
	motor[motorNum].ptCtrl.velOutput.knee = (motor[motorNum].ptCtrl.newPos.knee - motor[motorNum].posCtrl.actualPos.knee)/ (motor[motorNum].ptCtrl.desiredTime);			//做差值	
	motor[motorNum].ptCtrl.referSpeed.knee=(motor[motorNum].ptCtrl.NextPos.knee - motor[motorNum].ptCtrl.newPos.knee)/ (motor[motorNum].ptCtrl.desiredTime);
	
	motor[motorNum].ptCtrl.output.knee=MaxMinLimit((motor[motorNum].ptCtrl.velOutput.knee+motor[motorNum].ptCtrl.referSpeed.knee)/2.0f,motor[motorNum].ptCtrl.velLimit);
	returnSpeed.knee=motor[motorNum].ptCtrl.output.knee;

	return returnSpeed;
}

/**
* @brief 	录入点（弧度制角度）
* @param    pointAngle：输入点
* @param    motorNum:	电机位置
* @return   motor[motorNum].ptCtrl.output 速度输出
* @author 	ACTION
* @note
*/
legJoint PointInput(uint8_t motorNum,float pointAngleHip,float pointAngleNextHip,float pointAngleKnee,float pointAngleNextKnee)
{
	motor[motorNum].ptCtrl.newPos   = transfer(motorNum+1,pointAngleHip,pointAngleKnee);	//new data
	motor[motorNum].ptCtrl.NextPos   = transfer(motorNum+1,pointAngleNextHip,pointAngleNextKnee);	//new data
	motor[motorNum].velCtrl.sendSpeed=PTCtrlTransfer(motorNum);//PTCtrlTransfer(uint8_t motorNum,PTCtrlType *ptCtrl,PosCtrlType *posCtrl)
	motor[motorNum].ptCtrl.lastPos  = transfer(motorNum+1,pointAngleHip,pointAngleKnee);	
	
	return   motor[motorNum].velCtrl.sendSpeed; 		
}

/**
  * @brief  肩关节角度脉冲转角度
  * @note
  * @param  
  * @retval None
  */
float HipTrans(int32_t pulse)
{
	float angle=0;
	angle=128.75f*PI/180.0f-acosf((220.05f*220.05f+50.931f*50.931f-(-pulse/8192.0f*8.0f+183.0f)*(-pulse/8192.0f*8.0f+183.0f))/(2.0f*220.05f*50.931f));
	return angle;
}

/**
  * @brief  膝关节角度脉冲转角度
  * @note
  * @param  
  * @retval None
  */
float KneeTrans(int32_t pulse)
{
	float angle=0;
	angle=145.11f*PI/180.0f-acosf((231.355f*231.355f+50.0f*50.0f-(-pulse/8192.0f*8.0f+183.0f)*(-pulse/8192.0f*8.0f+183.0f))/(2.0f*231.355f*50.0f));
	return angle;
}

