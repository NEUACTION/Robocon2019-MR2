#include "lift.h"
#include "gpio.h"
#include "time.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "elmo.h"
#include "sensor.h"
#include "module_RED.h"
#include "light.h"
#include "app_cfg.h"
#include "pps.h"
#include "switch.h"
#include "dataProcess.h"
#include "vloop.h"
#define LIFT_UP      (0)
#define LIFT_WAIT    (1)

lift_ lift;
extern MotorType motor[4];
extern reset_ Reset;
extern leg_ legOne, legTwo, legThr, legFor;  
void liftInit(void)
{
    lift.cnt=0; 
    lift.delaytime=10;
    lift.lightSignal=LIFT_WAIT;
    lift.liftFlag=0;
    lift.pos=-293000;
    GPIO_Init_Pins(GPIOB,GPIO_Pin_10,GPIO_Mode_IN);
     
}
extern uint8_t gyroFlag;
extern uint8_t site;
extern quadrupedStatus_ quadrupedStatus;
uint8_t Lift(void)
{
	static uint8_t goCnt=0;
	delay_ms(6);
	GetJointPos();
	DataRecord(); 
	
	if(gyroFlag == 2)	
	{
		/*最终初始化完成，等待手动车触发，红场亮红灯，蓝场亮蓝灯*/
		if(site == RED)
			RedSite();
		else
			BlueSite();
			
		USART_OUT(UART4,(uint8_t *)"init is ok !\r\n");
		Correct();
		gyroFlag = 3;

	}
    lift.lightSignal= GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
	
	VelOutput(legOne.legPos.hip,legOne.legPos.hip,legOne.legPos.knee,legOne.legPos.knee,legOne.ID);
	VelOutput(legTwo.legPos.hip,legTwo.legPos.hip,legTwo.legPos.knee,legTwo.legPos.knee,legTwo.ID);
	VelOutput(legThr.legPos.hip,legThr.legPos.hip,legThr.legPos.knee,legThr.legPos.knee,legThr.ID);
	VelOutput(legFor.legPos.hip,legFor.legPos.hip,legFor.legPos.knee,legFor.legPos.knee,legFor.ID);

	if(lift.lightSignal==LIFT_UP)
        lift.cnt++;
	else
		lift.cnt=0;
	
	if(GO == 0)
		goCnt++;
	else
		goCnt=0;
	
	if( lift.cnt > lift.delaytime)
		lift.liftFlag=1;
	
    USART_OUT(UART4,(uint8_t *)"  %d %d %d ",(int)(quadrupedStatus.euler[0]*180.0f/PI*100),(int)(quadrupedStatus.euler[1]*180.0f/PI*100),(int)(quadrupedStatus.euler[2]*180.0f/PI*100));
	/*电机速度控制输出*/
	USART_OUT( UART4,(uint8_t *)"SV:%d %d %d %d %d %d %d %d ",\
	(int)(motor[0].velCtrl.sendSpeed.hip*100),(int)(motor[0].velCtrl.sendSpeed.knee*100),\
	(int)(motor[1].velCtrl.sendSpeed.hip*100),(int)(motor[1].velCtrl.sendSpeed.knee*100),\
	(int)(motor[2].velCtrl.sendSpeed.hip*100),(int)(motor[2].velCtrl.sendSpeed.knee*100),\
	(int)(motor[3].velCtrl.sendSpeed.hip*100),(int)(motor[3].velCtrl.sendSpeed.knee*100));
	/*读到的关节角度*/
	USART_OUT( UART4,(uint8_t *)" rp:%d %d %d %d %d %d %d %d %d %d %d %d\r\n ",\
	(int)(legOne.readPos.turn*180.f/PI*100),(int)(legOne.readPos.hip*180.0f/PI*100),(int)(legOne.readPos.knee*180.f/PI*100),\
	(int)(legTwo.readPos.turn*180.f/PI*100),(int)(legTwo.readPos.hip*180.0f/PI*100),(int)(legTwo.readPos.knee*180.f/PI*100),\
	(int)(legThr.readPos.turn*180.f/PI*100),(int)(legThr.readPos.hip*180.0f/PI*100),(int)(legThr.readPos.knee*180.f/PI*100),\
	(int)(legFor.readPos.turn*180.f/PI*100),(int)(legFor.readPos.hip*180.0f/PI*100),(int)(legFor.readPos.knee*180.f/PI*100));
	if(Reset.status == 4)
	{
		if(lift.liftFlag==1&&lift.cnt >= lift.delaytime && gyroFlag == 3 && goCnt > lift.delaytime)
		{
			return 1;
		}
		else
			return 0;
	}
	else
	{
		if(lift.liftFlag==1&&lift.cnt >= lift.delaytime && gyroFlag == 3)
		{
			return 1;
		}
		else
			return 0;
	}
}
