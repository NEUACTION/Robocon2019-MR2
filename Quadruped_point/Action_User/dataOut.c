#include "dataOut.h"
#include "pps.h"
#include "moveBase.h"
#include "vloop.h"
#include "balance.h"
#include "sensor.h"
#include "cloudplatform.h"
#include "pid.h"
#include "distanceAdjust.h"
#include "dma.h"
#include "module_RED.h"
#include "switch.h"

extern int can1ErrNum[8],can2ErrNum[4];
extern leg_ legOne, legTwo, legThr, legFor;  
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern MotorType motor[4];
extern visual_ visual;
extern BalancerType gBalancer1;
extern BalancerType gBalancer2;
extern float maxAngleRecord,minAngleRecord,distanceRecord;
extern uint16_t DebugUSARTSendBuffCnt;
extern uint8_t DebugUSARTSendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern uint8_t DebugUSARTDMASendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern uint16_t DebugUSARTSendBuffCnt2;
extern uint8_t DebugUSARTSendBuf2[DEBUG_USART_SEND_BUF_CAPACITY_2];
extern uint8_t DebugUSARTDMASendBuf2[DEBUG_USART_SEND_BUF_CAPACITY_2];
extern float desireAngle_X,desireAngle_Y;
extern float CPPos;
extern float rb_turnInit[N];
extern float lb_turnInit[N];
extern float rf_turnInit[N];
extern float lf_turnInit[N];

/** 
  * @brief  复位发数
  * @note
  * @param  None
  * @retval None 
  */
extern int order;
extern uint8_t gvro1ready;
extern uint8_t gvro2ready;
extern int correctCnt;
void ResetDataSendOut(void)
{
	/*can错误计数*/
	CANErrorNumber();
	
	/*电机速度控制输出*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"SV:%d %d %d %d %d %d %d %d ",\
	(int)(motor[0].velCtrl.sendSpeed.hip),(int)(motor[0].velCtrl.sendSpeed.knee),\
	(int)(motor[1].velCtrl.sendSpeed.hip),(int)(motor[1].velCtrl.sendSpeed.knee),\
	(int)(motor[2].velCtrl.sendSpeed.hip),(int)(motor[2].velCtrl.sendSpeed.knee),\
	(int)(motor[3].velCtrl.sendSpeed.hip),(int)(motor[3].velCtrl.sendSpeed.knee));
	
	/*电机速度控制输出*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"RV:%d %d %d %d %d %d %d %d ",\
	(int)(motor[0].ptCtrl.referSpeed.hip),(int)(motor[0].ptCtrl.referSpeed.knee),\
	(int)(motor[1].ptCtrl.referSpeed.hip),(int)(motor[1].ptCtrl.referSpeed.knee),\
	(int)(motor[2].ptCtrl.referSpeed.hip),(int)(motor[2].ptCtrl.referSpeed.knee),\
	(int)(motor[3].ptCtrl.referSpeed.hip),(int)(motor[3].ptCtrl.referSpeed.knee));
	
	/*电机速度控制输出*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"CV:%d %d %d %d %d %d %d %d ",\
	(int)(motor[0].ptCtrl.velOutput.hip),(int)(motor[0].ptCtrl.velOutput.knee),\
	(int)(motor[1].ptCtrl.velOutput.hip),(int)(motor[1].ptCtrl.velOutput.knee),\
	(int)(motor[2].ptCtrl.velOutput.hip),(int)(motor[2].ptCtrl.velOutput.knee),\
	(int)(motor[3].ptCtrl.velOutput.hip),(int)(motor[3].ptCtrl.velOutput.knee));
	
	/*给定腿关节角度信息*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"SP:%d %d %d %d %d %d %d %d %d %d %d %d ",\
	(int)(legOne.legPos.turn*180.f/PI*100),(int)(legOne.legPos.hip*180.0f/PI*100),(int)(legOne.legPos.knee*180.f/PI*100),\
	(int)(legTwo.legPos.turn*180.f/PI*100),(int)(legTwo.legPos.hip*180.0f/PI*100),(int)(legTwo.legPos.knee*180.f/PI*100),\
	(int)(legThr.legPos.turn*180.f/PI*100),(int)(legThr.legPos.hip*180.0f/PI*100),(int)(legThr.legPos.knee*180.f/PI*100),\
	(int)(legFor.legPos.turn*180.f/PI*100),(int)(legFor.legPos.hip*180.0f/PI*100),(int)(legFor.legPos.knee*180.f/PI*100));
	
//	/*给定腿关节角度信息*/
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"np:%d %d %d %d %d %d %d %d %d %d %d %d ",\
//    (int)(legOne.legNextPos.turn*180.f/PI*100),(int)(legOne.legNextPos.hip*180.0f/PI*100),(int)(legOne.legNextPos.knee*180.f/PI*100),\
//	(int)(legTwo.legNextPos.turn*180.f/PI*100),(int)(legTwo.legNextPos.hip*180.0f/PI*100),(int)(legTwo.legNextPos.knee*180.f/PI*100),\
//	(int)(legThr.legNextPos.turn*180.f/PI*100),(int)(legThr.legNextPos.hip*180.0f/PI*100),(int)(legThr.legNextPos.knee*180.f/PI*100),\
//	(int)(legFor.legNextPos.turn*180.f/PI*100),(int)(legFor.legNextPos.hip*180.0f/PI*100),(int)(legFor.legNextPos.knee*180.f/PI*100));
		
	/*读到的关节角度*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"RP:%d %d %d %d %d %d %d %d %d %d %d %d ",\
	(int)(legOne.readPos.turn*180.f/PI*100),(int)(legOne.readPos.hip*180.0f/PI*100),(int)(legOne.readPos.knee*180.f/PI*100),\
	(int)(legTwo.readPos.turn*180.f/PI*100),(int)(legTwo.readPos.hip*180.0f/PI*100),(int)(legTwo.readPos.knee*180.f/PI*100),\
	(int)(legThr.readPos.turn*180.f/PI*100),(int)(legThr.readPos.hip*180.0f/PI*100),(int)(legThr.readPos.knee*180.f/PI*100),\
	(int)(legFor.readPos.turn*180.f/PI*100),(int)(legFor.readPos.hip*180.0f/PI*100),(int)(legFor.readPos.knee*180.f/PI*100));
	
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"GYROPOS:%d %d ",(int)(gBalancer1.motor.posPulse),(int)(gBalancer2.motor.posPulse));
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d %d %d ",gvro1ready,gvro2ready,order,correctCnt);
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" \r\n");

}


/** 
  * @brief  发数
  * @note
  * @param  None
  * @retval None 
  */
void DataSendOut(void)
{
	SensorDataOut();
//	GetODOut();
	adjustDataOut();
//	ResetDataOut();
	/*can错误计数*/
    CANErrorNumber();
	MotorDataOut();
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" \r\n");
}

/** 
  * @brief  传感器的值
  * @note
  * @param  None
  * @retval None 
  */
void SensorDataOut(void)
{
	/*陀螺仪的值*/	
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"ops:%d %d %d %d %d %d ", \
	(int)(quadrupedStatus.euler[0]*180/PI*100),(int)(quadrupedStatus.euler[1]*180/PI*100),(int)(quadrupedStatus.euler[2]*180/PI*100),\
	(int)(quadrupedStatus.gyro[0]*180/PI*100),(int)(quadrupedStatus.gyro[1]*180/PI*100),(int)(quadrupedStatus.gyro[2]*180/PI*100));
	
	/*视觉距离和阶段*/	
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d %d ",(int)(CP.pitch*100),(int)(CP.roll*100), (int)(CP.yaw*100));
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"vis:%d %d %d %d %d %d ",(int)(visual.distance_C[0]*100),(int)(visual.distance_C[1]*100), (int)(visual.angle*100), (int)(visual.time),(int)visual.status,visual.goFlag);
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d ",(int)(visual.distanceNow[0]*100),(int)(visual.distanceNow[1]*100));
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d ",(int)(visual.compensate[0]*100),(int)(visual.compensate[1]*100));
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d ",(int)(quadrupedStatus.estimatedPos[0]),(int)(quadrupedStatus.estimatedPos[1]));
}

/** 
  * @brief  调节量的值
  * @note
  * @param  None
  * @retval None 
  */
void adjustDataOut(void)
{

	/*航行控制输出*/
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"ad:%d %d %d ",(int)(quadrupedControl.posAngle*180.0f/PI*100),(int)(quadrupedControl.yawAdjust*180.0f/PI*100),(int)(distanceRecord*100));
	
//	USART_OUT(UART4,(uint8_t *)"%d %d ",(int)(quadrupedControl.lf_rbAngleAdjust*180.0f/PI*100),(int)(quadrupedControl.rf_lbAngleAdjust*180.0f/PI*100));
	
	/*电机速度控制输出*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"SV:%d %d %d %d %d %d %d %d ",\
	(int)(motor[0].velCtrl.sendSpeed.hip),(int)(motor[0].velCtrl.sendSpeed.knee),\
	(int)(motor[1].velCtrl.sendSpeed.hip),(int)(motor[1].velCtrl.sendSpeed.knee),\
	(int)(motor[2].velCtrl.sendSpeed.hip),(int)(motor[2].velCtrl.sendSpeed.knee),\
	(int)(motor[3].velCtrl.sendSpeed.hip),(int)(motor[3].velCtrl.sendSpeed.knee));
	
//	/*电机速度控制输出*/
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"RV:%d %d %d %d %d %d %d %d ",\
//	(int)(motor[0].ptCtrl.referSpeed.hip),(int)(motor[0].ptCtrl.referSpeed),\
//	(int)(motor[1].ptCtrl.referSpeed.hip),(int)(motor[1].ptCtrl.referSpeed),\
//	(int)(motor[2].ptCtrl.referSpeed.hip),(int)(motor[2].ptCtrl.referSpeed),\
//	(int)(motor[3].ptCtrl.referSpeed.hip),(int)(motor[3].ptCtrl.referSpeed));
//	
//	/*电机速度控制输出*/
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"CV:%d %d %d %d %d %d %d %d ",\
//	(int)(motor[0].ptCtrl.velOutput.hip),(int)(motor[0].ptCtrl.velOutput),\
//	(int)(motor[1].ptCtrl.velOutput.hip),(int)(motor[1].ptCtrl.velOutput),\
//	(int)(motor[2].ptCtrl.velOutput.hip),(int)(motor[2].ptCtrl.velOutput),\
//	(int)(motor[3].ptCtrl.velOutput.hip),(int)(motor[3].ptCtrl.velOutput));
	
	
	/*给定腿关节角度信息*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"sp:%d %d %d %d %d %d %d %d %d %d %d %d ",\
    (int)(legOne.legPos.turn*180.f/PI*100),(int)(legOne.legPos.hip*180.0f/PI*100),(int)(legOne.legPos.knee*180.f/PI*100),\
	(int)(legTwo.legPos.turn*180.f/PI*100),(int)(legTwo.legPos.hip*180.0f/PI*100),(int)(legTwo.legPos.knee*180.f/PI*100),\
	(int)(legThr.legPos.turn*180.f/PI*100),(int)(legThr.legPos.hip*180.0f/PI*100),(int)(legThr.legPos.knee*180.f/PI*100),\
	(int)(legFor.legPos.turn*180.f/PI*100),(int)(legFor.legPos.hip*180.0f/PI*100),(int)(legFor.legPos.knee*180.f/PI*100));
		
	/*给定腿关节角度信息*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"np:%d %d %d %d %d %d %d %d %d %d %d %d ",\
  (int)(legOne.legNextPos.turn*180.f/PI*100),(int)(legOne.legNextPos.hip*180.0f/PI*100),(int)(legOne.legNextPos.knee*180.f/PI*100),\
	(int)(legTwo.legNextPos.turn*180.f/PI*100),(int)(legTwo.legNextPos.hip*180.0f/PI*100),(int)(legTwo.legNextPos.knee*180.f/PI*100),\
	(int)(legThr.legNextPos.turn*180.f/PI*100),(int)(legThr.legNextPos.hip*180.0f/PI*100),(int)(legThr.legNextPos.knee*180.f/PI*100),\
	(int)(legFor.legNextPos.turn*180.f/PI*100),(int)(legFor.legNextPos.hip*180.0f/PI*100),(int)(legFor.legNextPos.knee*180.f/PI*100));

}

/** 
  * @brief  读得电机的值
  * @note
  * @param  None
  * @retval None 
  */
void MotorDataOut(void)
{
    extern reset_ Reset;
//	/*读得足端相对肩关节位置*/
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" %d %d %d %d %d %d %d %d ",\
//	(int)(legOne.read.x*100),(int)(legOne.read.y*100),(int)(legTwo.read.x*100),(int)(legTwo.read.y*100),(int)(legThr.read.x*100),(int)(legThr.read.y*100),\
//	(int)(legFor.read.x*100),(int)(legFor.read.y*100));
		
	/*读到的关节角度脉冲*/
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t*)"%d %d %d %d %d %d %d %d ",\
//	(int)(motor[0].posCtrl.actualPos.hip),(int)(motor[0].posCtrl.actualPos.knee),\
//	(int)(motor[1].posCtrl.actualPos.hip),(int)(motor[1].posCtrl.actualPos.knee),\
//	(int)(motor[2].posCtrl.actualPos.hip),(int)(motor[2].posCtrl.actualPos.knee),\
//	(int)(motor[3].posCtrl.actualPos.hip),(int)(motor[3].posCtrl.actualPos.knee));
	
	/*读到的关节角度*/
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" rp:%d %d %d %d %d %d %d %d %d %d %d %d ",\
	(int)(legOne.readPos.turn*180.f/PI*100),(int)(legOne.readPos.hip*180.0f/PI*100),(int)(legOne.readPos.knee*180.f/PI*100),\
	(int)(legTwo.readPos.turn*180.f/PI*100),(int)(legTwo.readPos.hip*180.0f/PI*100),(int)(legTwo.readPos.knee*180.f/PI*100),\
	(int)(legThr.readPos.turn*180.f/PI*100),(int)(legThr.readPos.hip*180.0f/PI*100),(int)(legThr.readPos.knee*180.f/PI*100),\
	(int)(legFor.readPos.turn*180.f/PI*100),(int)(legFor.readPos.hip*180.0f/PI*100),(int)(legFor.readPos.knee*180.f/PI*100));
    
	
}

/** 
  * @brief  计算得到的相对于障碍物的距离
  * @note
  * @param  None
  * @retval None 
  */
void GetODOut(void)
{
	/*相对障碍物的距离*/
		
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" %d %d %d %d ",\
	(int)(legOne.D.stepOneD*100),(int)(legTwo.D.stepOneD*100),(int)(legThr.D.stepOneD*100),(int)(legFor.D.stepOneD*100));
	
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d %d %d ",\
	(int)(legOne.D.ropeOneD*100),(int)(legTwo.D.ropeOneD*100),(int)(legThr.D.ropeOneD*100),(int)(legFor.D.ropeOneD*100));
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d %d %d ",\
//	(int)(legOne.D.ropeTwoD*100),(int)(legTwo.D.ropeTwoD*100),(int)(legThr.D.ropeTwoD*100),(int)(legFor.D.ropeTwoD*100));
	
//	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d %d %d ",\
//	(int)(legOne.D.upSlopeD*100),(int)(legTwo.D.upSlopeD*100),(int)(legThr.D.upSlopeD*100),(int)(legFor.D.upSlopeD*100));

}


void ResetDataOut(void)
{


    extern uint8_t   startCnt;    
    extern uint8_t   strightStep;
    extern uint8_t   strightCnt;
    extern uint8_t   stepStep,stepCnt;    
    extern uint8_t   STRStep;
    extern uint8_t   STRCnt;    
    extern uint8_t   ropeStep;
    extern uint8_t   ropeCnt;    
    extern uint8_t   relayCnt;    
    extern uint8_t   hillStep;
	extern uint8_t   hillCnt;
    extern reset_ Reset;
    extern uint8_t runningMode;
    USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)
    "k:%d %d %d %d %d%d%d%d %d %d ",
    Reset.wait,
    Reset.waitCnt,
    (int)(Reset.button),(int)(Reset.lastButton),
    (int)Reset.statusButton1,(int)Reset.statusButton2,(int)Reset.statusButton3,(int)Reset.statusButton4,
    (int)Reset.status,runningMode);
     USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)
    "g:%d %d %d %d %d %d %d ",
    startCnt,strightCnt,stepCnt,STRCnt,strightStep,stepStep,STRStep);
        
}
/** 
  * @brief  错误计数
  * @note
  * @param  None
  * @retval None 
  */
extern int record[8];
void CANErrorNumber(void)
{
	USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" canError:%d %d %d %d %d %d %d %d %d %d %d %d ",\
	can1ErrNum[0],can1ErrNum[1],can1ErrNum[2],can1ErrNum[3],can1ErrNum[4],can1ErrNum[5],can1ErrNum[6],can1ErrNum[7],
	can2ErrNum[0],can2ErrNum[1],can2ErrNum[2],can2ErrNum[3]);

}

/** 
  * @brief  陀螺发数用串口5
  * @note
  * @param  None
  * @retval None 
  */
extern float M3508Speed[2];
extern float u1Rrcord,u2Rrcord;
extern float closeLoopRecord1,closeLoopRecord2;
extern float closeLoopForward_y,closeLoopForward_x;
void GyroDataOut(void)
{
 
	extern int gaitCnt;
	extern uint8_t goBack_flag_y,goBack_flag_x;
	extern uint8_t goBack_status ;
    extern uint8_t goBack_pos_x;
    extern uint8_t runningMode;
    extern float gyro1InitPos;
    extern float gyro2InitPos;
    extern uint8_t gyro1InitFlag ;
    extern uint8_t gyro2InitFlag ;
    extern uint8_t gvro1ready;
    extern uint8_t gvro2ready;
    extern Serial_ serial1;
    extern Serial_ serial2;

    extern float closeLoopForward_y ;
    extern float closeLoopForward_x ;

    USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"%d\t",(int)(gaitCnt));
    

	//y轴:右前和左后   6号3508控制
	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"6:%d %d ",(int)gBalancer1.motor.pos,(int)gBalancer1.motor.speed);
	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"Y:%d %d %d %d ",(int)(gBalancer1.sysPos.angle[1]*1000),(int)((gBalancer1.sysPos.angularVelocity[1]*100)),(int)serial1.outPut,(int)closeLoopForward_y);
    USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"O:%d %d %d ",(int)(serial1.wLoopIn*1000),(int)closeLoopRecord1,(int)(M3508Speed[0]));
                                                                                                                                               
    USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"7:%d %d ",(int)gBalancer2.motor.pos,(int)gBalancer2.motor.speed);
	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"X:%d %d %d %d ",(int)(gBalancer2.sysPos.angle[0]*1000),(int)((gBalancer2.sysPos.angularVelocity[0]*100)),(int)serial2.outPut,(int)closeLoopForward_x);
    USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"O:%d %d %d ",(int)(serial2.wLoopIn*1000),(int)closeLoopRecord2,(int)(M3508Speed[1]));
 
    
//    USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"%d %d %d %d ",runningMode,(int)gBalancer1.motor.pos,(int)gyro1InitPos,(int)gBalancer1.motor.speed);
//	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"%d %d %d ",(int)(gBalancer2.sysPos.angle[0]*1000),(int)gyro1InitFlag,(int)gvro1ready);
//    
//    
// 
//    
//	//x轴:左前和右后， 7号3508控制
//	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"%d %d %d",(int)gBalancer2.motor.pos,(int)gyro2InitPos,(int)gBalancer2.motor.speed);
//	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"%d %d %d",(int)(gBalancer2.sysPos.angle[0]*1000),(int)gyro2InitFlag,(int)gvro2ready);

	USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t *)"\r\n"); 
}



