#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "pps.h"
#include "moveBase.h"
#include "sensor.h"
#include "module_RED.h"
#include "vloop.h"
#include "cloudplatform.h"
#include "lift.h"
#include "light.h"
#include "dataProcess.h"
#include "pid.h"
#include "errorHand.h"
#include "init.h"
#include "dataOut.h"
#include "balance.h"
#include "dma.h"
#include "switch.h"


/*
===============================================================
						信号量定义
===============================================================
*/
OS_EXT INT8U OSCPUUsage;
OS_EVENT *PeriodSem;
OS_EVENT *ConfigPeriodSem;
OS_EVENT *BalancePeriodSem;

static OS_STK App_ConfigStk[Config_TASK_START_STK_SIZE];
static OS_STK WalkTaskStk[Walk_TASK_STK_SIZE];
static OS_STK BalanceTaskStk[Balance_TASK_STK_SIZE];

/*
===============================================================
						全局变量定义
===============================================================
*/
/*记录红蓝场变量*/
uint8_t gyroFlag=0;

/*记录红蓝场变量*/
uint8_t site=0;

/*模式切换变量*/
uint8_t runningMode=1;

/*定时器周期变量，单位为每一百微秒*/
uint16_t timeCount=200;

/*定时器周期记录变量，单位为每一百微秒*/
uint16_t timeRecord=200;

/*can1返回的errorcode*/
uint8_t can1ErrorCode=0;

/*can2返回的errorcode*/
uint8_t can2ErrorCode=0;

uint8_t slowFlag=0;

uint8_t opsInitkey=0;

uint8_t controlOrder=0;

uint8_t self_check=0;
extern leg_ legOne, legTwo, legThr, legFor;  
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus; 
extern visual_ visual;
extern uint8_t startOver;
extern uint8_t stage;
extern uint8_t correctCnt; 
extern uint16_t timeOut;
extern BalancerType gBalancer1;
extern BalancerType gBalancer2;
extern uint16_t gaitCnt;
extern uint16_t DebugUSARTSendBuffCnt;
extern uint8_t DebugUSARTSendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern uint8_t DebugUSARTDMASendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern uint16_t DebugUSARTSendBuffCnt2;
extern uint8_t DebugUSARTSendBuf2[DEBUG_USART_SEND_BUF_CAPACITY_2];
extern uint8_t DebugUSARTDMASendBuf2[DEBUG_USART_SEND_BUF_CAPACITY_2];
extern reset_ Reset;
extern uint8_t stage;
extern MotorType motor[4];
extern lift_ lift;
extern float desireAngle_X,desireAngle_Y;
extern int8_t STRacc;
void App_Task()
{
	CPU_INT08U os_err;
	os_err = os_err; /*防止警告...*/

	/*创建信号量   主程序的信号量*/
	PeriodSem = OSSemCreate(0);
    
    /*创建信号量   复位的信号量*/
	ConfigPeriodSem=OSSemCreate(0);
    
    
    
    /*创建信号量   陀螺平衡信号量*/
    BalancePeriodSem = OSSemCreate(0);
	
    
    /*创建任务*/
	os_err = OSTaskCreate((void (*)(void *))ConfigTask, /*初始化任务*/
						  (void *)0,
						  (OS_STK *)&App_ConfigStk[Config_TASK_START_STK_SIZE - 1],
						  (INT8U)Config_TASK_START_PRIO);
    //主任务
	os_err = OSTaskCreate((void (*)(void *))WalkTask,
						  (void *)0,
						  (OS_STK *)&WalkTaskStk[Walk_TASK_STK_SIZE - 1],
						  (INT8U)Walk_TASK_PRIO);
                          
   //陀螺平衡任务                         
	os_err = OSTaskCreate((void (*)(void *))BalanceTask,
						  (void *)0,
						  (OS_STK *)&BalanceTaskStk[Balance_TASK_STK_SIZE-1],
						  (INT8U)Balance_TASK_PRIO);
                          
                  
}

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
uint8_t beginBalance=0; 
int pwmNum = 0;
float pwmRank = 0;

void ConfigTask(void)
{
	CPU_INT08U os_err;
	os_err = os_err;
	
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USARTDMASendInit(UART4,DebugUSARTDMASendBuf,&UART4_Init,921600);
	USARTDMASendInit(UART5,DebugUSARTDMASendBuf2,&UART5_Init,921600);
	UART5_Init(921600);
	USART3_Init(115200);
	USART2_Init(115200);
	USART6_Init(115200); 
	TIM1_PWM_Init(20000-1,168-1);
	TIM4_PWM_Init(20000-1,84-1);
	TIM5_PWM_Init(20000-1,84-1);
    TIM8_PWM_Init(20000-1,168-1);
	
    TIM_Init(TIM2, 99, 83, 2, 0);// 0.1ms定时器中断
	TIM_Cmd(TIM2,DISABLE);	
	TIM_Init(TIM3, 99, 83, 0, 0);    
	
	TIM_SetCompare1(TIM1,1000); //PA8
    TIM_SetCompare3(TIM8,1000); //PC8
    TIM_SetCompare1(TIM4,1000); //PD12
    TIM_SetCompare3(TIM4,1000); //PD14   
	

    
	CAN_Config(CAN1,500,GPIOB,GPIO_Pin_8,GPIO_Pin_9);
	CAN_Config(CAN2,500,GPIOB,GPIO_Pin_5,GPIO_Pin_6);

   
	/*四条腿相应参数初始化*/
	LegParaInit();
	sreialPIDInit();
	/*抬升参数初始化*/
	liftInit();
	
	/*PID参数初始化*/
	PIDParaInit();
	
	/*开关初始化*/
	SwitchInit();
	
	resetInit();
	
    delay_ms(500);
    
    Reset.button= GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);
    self_check=GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1);    
	lift.lightSignal= GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);

	Reset.statusButton1= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6);
	Reset.statusButton2= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5);
	Reset.statusButton3= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4);
	Reset.statusButton4= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3);


	/*检测红蓝场是否正确，红场亮红灯，蓝场亮蓝灯*/
	if(SELECT_SITE == RED)
	{
		site=RED;
		RedSite();
	}
	else
	{
		site=BLUE;
		BlueSite();
	}
	if(Reset.button || Reset.statusButton4||Reset.statusButton1||Reset.statusButton2||Reset.statusButton3)
		BrightPurple();
    if(self_check==1)
    {
        White(LEGONE_LIGHT_ID);
        White(LEGTWO_LIGHT_ID);
        White(LEGTHR_LIGHT_ID);
        White(LEGFOR_LIGHT_ID);
        WaitOpsPrepare();  
    }   
	if(Reset.button == 1 && Reset.statusButton4 == 1)
	{
		timeCount=180;
		timeRecord=180;
		/*直接进入上坡*/
		stage=UPHILL;
		gaitCnt=31;
		Reset.status=4;
		runningMode=4;
		legOne.readPos.hip=83.0f*PI/180.0f;
		legTwo.readPos.hip=83.0f*PI/180.0f;
		legThr.readPos.hip=83.0f*PI/180.0f;
		legFor.readPos.hip=83.0f*PI/180.0f;
		
		legOne.readPos.knee=119.763f*PI/180.0f;
		legTwo.readPos.knee=119.763f*PI/180.0f;
		legThr.readPos.knee=119.763f*PI/180.0f;
		legFor.readPos.knee=119.763f*PI/180.0f;
		
		legOne.readPos.turn=-95.0f*PI/180.0f;
		legThr.readPos.turn=-95.0f*PI/180.0f;
		legTwo.readPos.turn=95.0f*PI/180.0f;
		legFor.readPos.turn=95.0f*PI/180.0f;
		
		 motor[0].ptCtrl.lastPos = transfer(1,83.39f*PI/180.0f,119.79f*PI/180.0f);	
         motor[1].ptCtrl.lastPos = transfer(2,83.39f*PI/180.0f,119.79f*PI/180.0f);	
         motor[2].ptCtrl.lastPos = transfer(3,83.39f*PI/180.0f,119.79f*PI/180.0f);	
         motor[3].ptCtrl.lastPos = transfer(4,83.39f*PI/180.0f,119.79f*PI/180.0f);
		
		if(site==RED)
			ReStartInit(47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f);
		if(site==BLUE)
			ReStartInit(47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f);
			
	}
	else
	{
		timeCount=160;
		timeRecord=160;
		/*正常启动*/
		StartInit(35.4226f,90.122f,35.4226f,90.122f,35.4226f,90.122f,35.4226f,90.122f);
	}

	/*电机配置初始化*/
	MoterInit();
	OSSemSet(ConfigPeriodSem, 0, &os_err);
	while(!startOver||self_check==1)
	{
		OSSemPend(ConfigPeriodSem, 0, &os_err);	
		if(self_check==1)
            readGPIO();
		balanceCtrlInit();
        /*读电机角度*/
		GetJointPos();
		GetGyroMessage();
		/*记录足端位置*/
		DataRecord(); 
		/*如果can通讯中断就将能抱死的电机抱死*/
		if(CanCommunicateErr() == 1)
			CanErrDisplay();
		else if(self_check==1)
        {
            if(startOver==0)
                PosInit();
            else 
                Stop();
            GyroSelfCheck();
        }
        else if(self_check==0)
            PosInit();
        
		/*接收到can1发回来的canerrornum*/
		can1ErrorCode=CAN_GetLastErrorCode(CAN1);
		
		/*接收到can2发回来的canerrornum*/
		can2ErrorCode=CAN_GetLastErrorCode(CAN2);
		
		/*串口发数*/
		USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d ",(int)(can1ErrorCode),(int)(can2ErrorCode));
		ResetDataSendOut();
        
	}
	
	/*速度给0，确定完全抱死*/
	Stop();
	
	/*压低复位限幅*/
	motor[0].ptCtrl.velLimit  = GROUND_LIMIT;	
	motor[1].ptCtrl.velLimit  = GROUND_LIMIT;	
	motor[2].ptCtrl.velLimit  = GROUND_LIMIT;	
	motor[3].ptCtrl.velLimit  = GROUND_LIMIT;	
	
	/*青色的灯指示姿态初始化完成*/
	BrightBlue_green();
	
	/*一直等待定位系统初始化完成*/
	WaitOpsPrepare();
   
	/*粉色的灯指示定位系统初始化完成*/
	BrightPurple();
	delay_ms(500);
    
    #ifdef RUNNING_MODE
	/*陀螺油门*/
    pwmRank = 40;
    #else 
    pwmRank= 0;
    #endif
    
    
	pwmNum = 1000 + pwmRank * (2000.f - 1000.f) / 100.f;
	
	/*开启陀螺*/
    TIM_SetCompare1(TIM1,(pwmNum - 1)); //PA8       7 3508    (左后，俯视顺时针)
    TIM_SetCompare3(TIM8,(pwmNum - 1)); //Pc8       7 3508    (左前，俯视顺时针)
    TIM_SetCompare1(TIM4,(pwmNum - 1)); //PD12      6 3508    (右后, 俯视顺时针)
    TIM_SetCompare3(TIM4,(pwmNum - 1)); //PD14      6 3508    (右前，俯视顺时针)   
	
	gyroFlag=1;
	
	/*等待手动车触发光电开关*/
	while(!Lift());
	
	/*清除复位标志位*/
	Reset.status=0;
	
	/*清除复位标志位*/
	runningMode=1;
	
	/*改回限幅*/
	motor[0].ptCtrl.velLimit  = NORMAL_LIMIT;	
	motor[1].ptCtrl.velLimit  = NORMAL_LIMIT;	
	motor[2].ptCtrl.velLimit  = NORMAL_LIMIT;	
	motor[3].ptCtrl.velLimit  = NORMAL_LIMIT;	

	
	TIM_Cmd(TIM2,ENABLE);
	OSTaskSuspend(OS_PRIO_SELF);
}
/*
   ===============================================================
   运动总任务
   ===============================================================
   */

extern uint16_t timeOut;
extern int testTime;
extern uint8_t testFlag;
extern int16_t timeAdd;
void WalkTask(void)
{ 

	CPU_INT08U os_err;
	os_err = os_err;
    if(site==RED)
	{
       if(gaitCnt >=2 && gaitCnt <=10)
		{
			desireAngle_X=0.70f;
			desireAngle_Y=1.8f;  
		}
		else
		{
			desireAngle_X=0.35f;
			desireAngle_Y=0.5f;  
		}	
	}
    else 
    {
		if(gaitCnt >=2 && gaitCnt <=10)
		{
			desireAngle_X=0.80f;
			desireAngle_Y=1.7f;  
		}
		else
		{
			desireAngle_X=0.35f;
			desireAngle_Y=0.5f;  
		}	
              
    }
    #ifdef RUNNING_MODE
	timeCount=150;
	timeRecord=150;
    #else 
    timeCount=300;
	timeRecord=300;
    #endif
    
	OSSemSet(PeriodSem, 0, &os_err);
	while (1)
    {
		OSSemPend(PeriodSem, 0, &os_err);
		
        if(runningMode == 1)
			/*速度改变*/
            #ifdef RUNNING_MODE
                SpeedPlan(&timeRecord,&timeCount);
            #endif
		timeOut=0;
        
		/*读关节位置*/
        GetJointPos();
		
        Reset.button= GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);
        Reset.statusButton1= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6);
        Reset.statusButton2= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5);
        Reset.statusButton3= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_4);
        Reset.statusButton4= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3);
        
        Reset.lightSwitch=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
		
		switch(runningMode)
		{

			case 1:
				
				startOver=0;
				/*每隔一段时间给小电脑发一次通讯是否正常*/
				CheckVisionCommunication(gaitCnt,quadrupedStatus.euler[2],timeRecord);
				
				/*偏航角控制*/
				if(gaitCnt !=0&&gaitCnt !=31)
					quadrupedControl.yawAdjust=YawControl(quadrupedStatus.euler[2],0.0f);
				else
					quadrupedControl.yawAdjust=0.0f;
				/*接收到can发回来的canerrornum*/
				can1ErrorCode=CAN_GetLastErrorCode(CAN1);
				/*接收到can2发回来的canerrornum*/
				can2ErrorCode=CAN_GetLastErrorCode(CAN2);
				/*如果can通讯中断就将能抱死的电机抱死*/
				if((CanCommunicateErr() == 1 || ((RolloverProtect(quadrupedStatus.euler) == 1))||TokenErr()==1)&&gaitCnt!=44)
				{
					if( RolloverProtect(quadrupedStatus.euler) == 1||TokenErr()==1)
						runningMode=2;
					if(CanCommunicateErr() == 1)
						CanErrDisplay();
				}
				else
				{	
					if(site == RED)
						walk_RED();	
					else if(site == BLUE)
						walk_BLUE();	
				}
				ResetScan();
                /*LED灯灭*/
                GPIO_SetBits(GPIOC, GPIO_Pin_13);
				break;
			 /*按下重启开关*/   
			case 2:
                MotorAllStop();
				MotorOff(CAN2, LIFT_UP_ID);
            	WhiteSite();
				DataCleanUp();
				DataRecord();
				ResetStatusSelect();
				if(Reset.status!=0&&Reset.lastStatus==0)
					runningMode=3;
                
                /*LED灯灭*/
                GPIO_SetBits(GPIOC, GPIO_Pin_13);
				break;
			/*按下选择开关*/    
			case 3:
                /*LED灯灭*/
                GPIO_SetBits(GPIOC, GPIO_Pin_13);
            
            	if(Reset.status==1)
					AllGreen(); 
				if(Reset.status==4)
					BrightPink();
                if(Reset.status==3)
                    BrightPurple();
                
				DataRecord();
				motorReInit();
				MotorAllStop();
				break;
			
			/*电机复位*/
			case 4:
				if(Reset.status==1)
				{
					if(Reset.statusButton1==0)
						runningMode=2;
				}
				
				if(Reset.status==4)
				{
					if(Reset.statusButton4==0)
						runningMode=2;
				}
                if(Reset.status==3)
				{
					if(Reset.statusButton3==0)
						runningMode=2;
                    
				}
				
				DataRecord();
				if(((Reset.statusButton1==1&&Reset.statusButton4==0&&Reset.statusButton3==0)||
                    (Reset.statusButton1==0&&Reset.statusButton4==0&&Reset.statusButton3==0)||
                    (Reset.statusButton1==0&&Reset.statusButton4==1&&Reset.statusButton3==0)||
                    (Reset.statusButton1==0&&Reset.statusButton4==0&&Reset.statusButton3==1))
                  )
					PosInit();
				else runningMode=2;
				Reset.wait=0;
				Reset.waitCnt=0;
                Reset.waitKey=0;
				slowFlag=0;
				break;
			
			
			/*按回重启开关，准备出发*/
			case 5:
                /*LED灯亮*/
				motor[0].ptCtrl.velLimit  = GROUND_LIMIT;	
				motor[1].ptCtrl.velLimit  = GROUND_LIMIT;	
				motor[2].ptCtrl.velLimit  = GROUND_LIMIT;	
				motor[3].ptCtrl.velLimit  = GROUND_LIMIT;	
			
				VelOutput(legOne.legPos.hip,legOne.legPos.hip,legOne.legPos.knee,legOne.legPos.knee,legOne.ID);
				VelOutput(legTwo.legPos.hip,legTwo.legPos.hip,legTwo.legPos.knee,legTwo.legPos.knee,legTwo.ID);
				VelOutput(legThr.legPos.hip,legThr.legPos.hip,legThr.legPos.knee,legThr.legPos.knee,legThr.ID);
				VelOutput(legFor.legPos.hip,legFor.legPos.hip,legFor.legPos.knee,legFor.legPos.knee,legFor.ID);
				if(Reset.button==1)
				{
					if(Reset.status==1)
					{
						if(Reset.statusButton1==0)
							runningMode=2;
					}
				
					if(Reset.status==4)
					{
						if(Reset.statusButton4==0)
							runningMode=2;
					}
                    if(Reset.status==3)
                    {
                        if(Reset.statusButton3==0)
							runningMode=2;
                    }
                    if(!((Reset.statusButton1==1&&Reset.statusButton4==0&&Reset.statusButton3==0)||
                    (Reset.statusButton1==0&&Reset.statusButton4==0&&Reset.statusButton3==0)||
                    (Reset.statusButton1==0&&Reset.statusButton4==1&&Reset.statusButton3==0)||
                    (Reset.statusButton1==0&&Reset.statusButton4==0&&Reset.statusButton3==1)))
                        runningMode=2;                    
                    
                    
				}					
				if(Reset.statusButton1&&Reset.statusButton4&&Reset.statusButton3)
					runningMode=2;
			
				PosCrl2(CAN2, LIFT_UP_ID ,ABSOLUTE_MODE,0);
				if(site==RED)
					RedSite();
				else BlueSite();
				
				MotorAllStop();
				DataRecord();
				
				/*开关触发*/
				if(Reset.wait==1&&Reset.button==0)
						Reset.waitCnt++;
				if(Reset.waitCnt>1&&Reset.waitKey==0)
                {
                    MotorOn(CAN2,LIFT_UP_ID);
                    Reset.waitKey=1;
                    Reset.waitCnt=0;
                    Reset.wait=0;
                    Correct();
					if(Reset.status==4)
						slowFlag=1;
                    if(Reset.status==3)
                        STRacc=1;
				}
				if(Reset.button==0&&Reset.lastButton==1)
						Reset.wait=1;
				
                
                if(Reset.waitKey==1)
                {
                    /*光电触发*/
                    if(Reset.waitCnt>=5)
                    {	
						motor[0].ptCtrl.velLimit  = NORMAL_LIMIT;	
						motor[1].ptCtrl.velLimit  = NORMAL_LIMIT;	
						motor[2].ptCtrl.velLimit  = NORMAL_LIMIT;	
						motor[3].ptCtrl.velLimit  = NORMAL_LIMIT;	
                        runningMode=1;

                    }
                    if(Reset.lightSwitch==0)
                            Reset.waitCnt++;                    
                }

				break;
			default: 
				break;
		}
		Reset.lastButton=Reset.button;
		Reset.lastStatus=Reset.status;
		
		/*数据转化*/
		DataConversion();

//		USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)" %d ",runningMode);
		USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d %d %d %d %d ",gaitCnt,timeRecord,timeAdd,(int)(can1ErrorCode),(int)(can2ErrorCode));
		DataSendOut();
		USARTDMAOUT( UART4,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"%d ",timeOut);
		timeOut=0;
	}
}

/*
==================================================================
陀螺平衡任务
6号3508电机控制右侧两个陀螺，调整Y轴（右前和左后对角线）
7号3508电机控制左侧两个陀螺，调整X轴（左前和右后对角线）
结构体gbalance1对应6号电机
结构体gbalance2对应7号电机
==================================================================
*/

float testW1=0;
float testW2=0;

extern uint8_t closeLoopFlag;
extern float openLoopVel1,openLoopVel2;
extern int openLoopPeriod;  
extern uint16_t timeOut2;
extern float M3508Speed[2];
uint8_t start_flag_y = 0,start_flag_x = 0;
uint8_t goBack_flag_y = 0, goBack_flag_x = 0,goBack_pos_y = 0, goBack_pos_x = 0;
void BalanceTask(void)
{
  	/*5ms控制周期*/
	CPU_INT08U os_err;
	os_err = os_err;
	
	/*闭环开始调节*/
	closeLoopFlag=1;
	

	OSSemSet(BalancePeriodSem, 0, &os_err);
	while (1)	 		
	{
        OSSemPend(BalancePeriodSem, 0, &os_err);
		timeOut2=0;
		GetGyroMessage();
        #ifdef RUNNING_MODE
		switch( runningMode)
		{
			case 1:
			{
				if((CanCommunicateErr() == 1 || RolloverProtect(quadrupedStatus.euler) == 1||TokenErr()==1) && gaitCnt!=44)
				{
					VelCrl(CAN2,3,0,0);
				}
				else
				{
					if(goBack_flag_y == 0)
						BalanceOpenLoopControl_y();//6号，控制y轴
					
					GyroBack_y(goBack_pos_y,&goBack_flag_y);
					
					if(goBack_flag_x == 0)
						BalanceOpenLoopControl_x();//7号，控制x轴
					
					GyroBack_x(goBack_pos_x,&goBack_flag_x);
					
					VelCrl(CAN2,3,(int)(RAD_TO_PULSE(M3508Speed[0])),(int)(RAD_TO_PULSE(M3508Speed[1])));

				}
				

			}  break;
			default: break;                
		}	
        #endif
		GyroDataOut();
		USARTDMAOUT( UART5,DebugUSARTSendBuf2,&DebugUSARTSendBuffCnt2,DebugUSARTDMASendBuf2,DEBUG_USART_SEND_BUF_CAPACITY_2,(uint8_t*)"%d\t",timeOut2);
		timeOut2=0;
    }
}

/*
==================================================================================
平衡陀螺控制率
陀螺向下转，3508顺时针转，陀螺逆时针转，使摆正方向转
左前方角动量朝下 左后方角动量朝上
右前方角动量朝下 做后方角动量朝上
==================================================================================
*/
