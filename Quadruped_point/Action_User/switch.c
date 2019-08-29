#include "switch.h"
#include "init.h"
#include "module_RED.h"
#include "pid.h"
#include "pps.h"
#include "light.h"
#include "lift.h"
#include "balance.h"
/** 
  * @brief  开关初始化
  * @note
  * @param  None
  * @retval None
  */
uint8_t resetSigal=0;
uint8_t lastresetSignal=0;
reset_ Reset;


/*步态相关变量*/
extern uint8_t stage;
extern uint8_t changFlag;
extern uint8_t hillStep;
extern uint8_t hillCnt;
extern uint8_t relayCnt;
extern uint8_t ropeStep;
extern uint8_t ropeCnt;
extern uint8_t STRStep;
extern uint8_t STRCnt; 
extern uint8_t stepStep;
extern uint8_t stepCnt;
extern uint8_t strightStep;
extern uint8_t strightCnt;
extern uint8_t startCnt;
extern uint16_t gaitCnt;
extern uint8_t runningMode;
extern uint16_t timeRecord;
extern BalancerType gBalancer1;
extern BalancerType gBalancer2;
/*1、3号腿支撑时陀螺闭环参数*/
extern gyro_t lf_rb;
extern float openLoopVel1;

/*2、4号腿支撑时陀螺闭环参数*/
extern gyro_t rf_lb;
extern float openLoopVel2;





void SwitchInit(void)
{
	/*红蓝场*/
	GPIO_Init_Pins(GPIOD,GPIO_Pin_7,GPIO_Mode_IN);
	
	/*重启点1，在起始点重启*/
	GPIO_Init_Pins(GPIOD,GPIO_Pin_6,GPIO_Mode_IN);
	
	/*重启点2，在山脚驿站重启*/
	GPIO_Init_Pins(GPIOD,GPIO_Pin_5,GPIO_Mode_IN);
	
	/*自检*/
	GPIO_Init_Pins(GPIOD,GPIO_Pin_4,GPIO_Mode_IN);
	
	/*陀螺仪清零*/
	GPIO_Init_Pins(GPIOD,GPIO_Pin_3,GPIO_Mode_IN);
	
	/*待定*/
	GPIO_Init_Pins(GPIOD,GPIO_Pin_1,GPIO_Mode_IN);
	
	/*重启初始完毕，开始走的开关*/
	GPIO_Init_Pins(GPIOE,GPIO_Pin_4,GPIO_Mode_IN);
	
	/*判断上坡的光电*/
	GPIO_Init_Pins(GPIOB,GPIO_Pin_14,GPIO_Mode_IN);
    
	GPIO_Init_Pins(GPIOC,GPIO_Pin_13,GPIO_Mode_OUT);
    GPIO_SetBits(GPIOC, GPIO_Pin_13);
}



/** 
  * @brief  光电开关触发上坡信号
  * @note
  * @param  None
  * @retval None
  */
uint8_t Manualtrigger(void)
{
	static uint8_t manualCnt=0;
	
	if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14) == 0)
	{
		manualCnt++;
        if(manualCnt==5)
            Correct();
	}
	else
		manualCnt=0;

	if(manualCnt > 10)
		return 1;
	else
		return 0;
}


void resetInit(void)
{
    Reset.button=0;
    Reset.lastButton=0;
    Reset.lastStatus=0;
    Reset.restart=0;
    Reset.restartOver=0;
    Reset.status=0;
    Reset.statusButton1=0;
    Reset.statusButton2=0;
    Reset.statusButton3=0;
    Reset.statusButton4=0;
    Reset.wait=0;
    Reset.waitCnt=0;
    
    
}


extern float closeLoopForward_y,closeLoopForward_x;
void  ResetStatusSelect(void)
{
#ifdef RUNNING_MODE
    if(Reset.statusButton1==1&&Reset.statusButton4==0&&Reset.statusButton3==0)
    {
		timeRecord=150;
        Reset.status=1;
        stage=START;
        gaitCnt=0;
        closeLoopForward_x = 0.f;
        closeLoopForward_y = 0.f;
    }
    else if(Reset.statusButton4&&Reset.statusButton1==0&&Reset.statusButton3==0)
    {       
		timeRecord=140;
        Reset.status=4;
        stage=UPHILL;
        gaitCnt=31;
        closeLoopForward_x = 0.f;
        closeLoopForward_y = 0.f;
    }
    else if(Reset.statusButton3&&Reset.statusButton2==0&&Reset.statusButton1==0)
    {
		timeRecord=150;
        Reset.status=3;
        stage=STEP_TO_ROPE;
        STRStep=1;
        gaitCnt=19;
        closeLoopForward_x = 0.f;
        closeLoopForward_y = 0.f;
        
    }
#else 
     if(Reset.statusButton1==1&&Reset.statusButton4==0&&Reset.statusButton3==0)
    {
		timeRecord=300;
        Reset.status=1;
        stage=START;
        gaitCnt=0;
        openLoopVel1 = 0.f;
        openLoopVel2 = 0.f;
    }
    else if(Reset.statusButton4&&Reset.statusButton1==0&&Reset.statusButton3==0)
    {       
		timeRecord=300;
        Reset.status=4;
        stage=UPHILL;
        gaitCnt=31;
        openLoopVel1 = 0.f;
        openLoopVel2 = 0.f;
    }
    else if(Reset.statusButton3&&Reset.statusButton2==0&&Reset.statusButton1==0)
    {
		timeRecord=300;
        Reset.status=3;
        stage=STEP_TO_ROPE;
        STRStep=1;
        gaitCnt=19;
        openLoopVel1 = 0.f;
        openLoopVel2 = 0.f;
        
    }
    #endif
    else  Reset.status=0;

    
}

extern uint8_t closeLoopFlag;
extern int order;
void DataCleanUp(void)
{
    extern uint8_t can1ErrNum[8];
    extern uint8_t protectFlag;
    extern int can2ErrNum[4];

    extern uint8_t gvro1ready;
    extern uint8_t gvro2ready;
    extern uint8_t out;
    extern visual_ visual;
    extern uint8_t tokenLost;
	extern uint8_t gyro2InitFlag ;
	extern uint8_t gyro1InitFlag ;	
	extern uint8_t start_flag_y ;
	extern uint8_t start_flag_x ;
    order=0;
    changFlag=0;
    hillStep=0;
    hillCnt=0;
    relayCnt=0;
    ropeStep=0;
    ropeCnt=0;
    STRStep=0;
    STRCnt=0; 
    stepStep=0,
    stepCnt=0;
    strightStep=0;
    strightCnt=0;
    startCnt=0;
    can1ErrNum[0]=0;
    can1ErrNum[1]=0;
    can1ErrNum[2]=0;
    can1ErrNum[3]=0;
    can1ErrNum[4]=0;
    can1ErrNum[5]=0;
    can1ErrNum[6]=0;
    can1ErrNum[7]=0;
    
    can2ErrNum[0]=0;
    can2ErrNum[1]=0;
    can2ErrNum[2]=0;
    can2ErrNum[3]=0;
    
    tokenLost=0;
    
    out=0;
    
    visual.goFlag =0; 
    
    
   /*陀螺数据初始化*/

	gyro2InitFlag = 0;
	gyro1InitFlag = 0;
	gvro2ready=0;
	gvro1ready=0;
	start_flag_x=0;
	start_flag_y=0;
	closeLoopFlag=1;
}
extern float openLoopVel1;
extern float openLoopVel2;
extern float openLoopDis1;
extern float openLoopDis2;
extern uint8_t goBack_flag_y, goBack_flag_x;
void motorReInit(void)
{
    extern uint8_t site;
	order=0;
    switch(Reset.status)
    {
		
        /*起点启动*/
        case 1:
			
            ReStartInit(35.4226f,90.122f,35.4226f,90.122f,35.4226f,90.122f,35.4226f,90.122f);
			openLoopVel1 = 0.f;
			openLoopVel2 = 0.f;
			
			lf_rb.k=2600.0f;
			lf_rb.b=650.0f;
			lf_rb.limit=270.0f;
			
			rf_lb.k=2600.0f;
			rf_lb.b=650.0f;
			rf_lb.limit=270.0f;
		
			goBack_flag_y=0;
			goBack_flag_x=0;
        break;

        case 3:
        if(site==RED)
            ReStartInit(54.4545,74.4871,14.9793,75.6993,14.9793,75.6993,54.4545,74.4871);
        else
            ReStartInit(14.9793,75.6993,54.4545,74.4871,54.4545,74.4871,14.9793,75.6993);
        
            goBack_flag_y=0;
            goBack_flag_x=0;
			openLoopDis1=0;
			openLoopDis2=0;
        break;

        case 4:
		if(site==RED)
			ReStartInit(47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f);
		if(site==BLUE)
			ReStartInit(47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f,47.0346f,104.3366f);
            goBack_flag_y=0;
			goBack_flag_x=0;
			openLoopDis1=0;
			openLoopDis2=0;
        break;
        default:break; 
            
            
    }
}

void ResetScan(void)
{
     if(Reset.wait==1&&Reset.button==1)
    {
        Reset.waitCnt++;
    }else Reset.waitCnt=0;
    
    if(Reset.waitCnt>1&&Reset.wait==1)
    {
        Reset.wait=0;
        Reset.waitCnt=0;
        runningMode=2;
    }else;
    if(Reset.button==1&&Reset.lastButton==0)
    {
        Reset.wait=1;
    }        
}


uint8_t frontLight=0;
void readGPIO(void)
{
    extern lift_ lift;
    extern uint8_t selfCheckKey;
    
    static uint8_t cnt=0;
    Reset.button= GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_4);
	Reset.statusButton4= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_5);
	Reset.statusButton1= GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_6);
	lift.lightSignal= GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
    frontLight=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14);
    if(frontLight==0||Reset.statusButton4==1||Reset.statusButton1||Reset.button==1)
    {
       AllGreen();
    }
    if(lift.lightSignal==0&&cnt<=50)
        cnt++;    
    if(cnt>50)
        PosCrl2(CAN2, LIFT_UP_ID ,ABSOLUTE_MODE,-275000);
    
}


void GyroSelfCheck(void)
{
    extern int pwmNum ;
    extern float pwmRank ;
    static int8_t gyro1backFlag=1;
    static int8_t gyro2backFlag=1;
    static int8_t gyro1lastbackFlag=1;
    static int8_t gyro2lastbackFlag=1;
    static uint8_t time=0;
    
    pwmRank = 20;
	pwmNum = 1000 + pwmRank * (2000.f - 1000.f) / 100.f;
    
    
    if(gBalancer1.motor.posPulse >=GYRO1_PROTECT_MAX)
        gyro1backFlag=-1;
    if(gBalancer1.motor.posPulse <=GYRO1_PROTECT_MIN)
        gyro1backFlag=1;
    if((gyro1backFlag==1&&gyro1lastbackFlag==-1)||(gyro2backFlag==1&&gyro2lastbackFlag==-1))
    {
        if(time<4)
        {
            TIM_SetCompare1(TIM1,(pwmNum - 1));
            TIM_SetCompare3(TIM8,1000);
            TIM_SetCompare1(TIM4,1000);
            TIM_SetCompare3(TIM4,1000);
            Purple(LEGTWO_LIGHT_ID);
            Gold(LEGONE_LIGHT_ID);
            Gold(LEGTHR_LIGHT_ID);
            Gold(LEGFOR_LIGHT_ID);            

        }
        if(time>=4&&time<8)
         {
            TIM_SetCompare1(TIM1,1000);
            TIM_SetCompare3(TIM8,(pwmNum - 1));
            TIM_SetCompare1(TIM4,1000);
            TIM_SetCompare3(TIM4,1000);
            Purple(LEGONE_LIGHT_ID);
            Gold(LEGTWO_LIGHT_ID);
            Gold(LEGTHR_LIGHT_ID);
            Gold(LEGFOR_LIGHT_ID);
        }
        if(time==8&&time<12)
         {
            TIM_SetCompare1(TIM1,1000);
            TIM_SetCompare3(TIM8,1000);
            TIM_SetCompare1(TIM4,(pwmNum - 1));
            TIM_SetCompare3(TIM4,1000);
            Purple(LEGTHR_LIGHT_ID);
            Gold(LEGONE_LIGHT_ID);
            Gold(LEGTWO_LIGHT_ID);
            Gold(LEGFOR_LIGHT_ID);
        }
        if(time>=12&&time<16)
         {
            TIM_SetCompare1(TIM1,1000);
            TIM_SetCompare3(TIM8,1000);
            TIM_SetCompare1(TIM4,1000);
            TIM_SetCompare3(TIM4,(pwmNum - 1));
            Purple(LEGFOR_LIGHT_ID);
            Gold(LEGONE_LIGHT_ID);
            Gold(LEGTWO_LIGHT_ID);
            Gold(LEGTHR_LIGHT_ID);
        }
         time++;
         if(time==16)
             time=0;
    }
    if(gyro1backFlag== 1) 
        BalancerMotorControl(50,1);
    if(gyro1backFlag==-1)
        BalancerMotorControl(-50,1);
    
    
    if(gBalancer2.motor.posPulse >=GYRO2_PROTECT_MAX)
        gyro2backFlag=-1;
    if(gBalancer2.motor.posPulse <=GYRO2_PROTECT_MIN)
        gyro2backFlag=1;
    if(gyro2backFlag==1) 
        BalancerMotorControl(50,2);
    if(gyro2backFlag==-1)
        BalancerMotorControl(-50,2);
    
    
    gyro1lastbackFlag=gyro1backFlag;
    gyro2lastbackFlag=gyro2backFlag;
    
    
//    PosCrl(CAN2,5,ABSOLUTE_MODE,-270000);
//    
}
