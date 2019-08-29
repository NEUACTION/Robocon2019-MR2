#include "init.h"
#include "vloop.h"
#include "balance.h"
#include "app_cfg.h"
#include "module_RED.h"
#include "cloudplatform.h"
#include "switch.h"

/*姿态复位完成标志位*/
uint8_t startOver=0;

/*姿态复位完成等待计数变量*/
int correctCnt=0;
#define RESETnum 100

extern leg_ legOne, legTwo, legThr, legFor;  
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern MotorType motor[4];
extern uint8_t site;
extern reset_ Reset;
extern uint8_t runningMode;
extern BalancerType gBalancer1;
extern BalancerType gBalancer2;
/** 
  * @brief  四足腿部关节初始化
  * @note
  * @param  None
  * @retval None
  */
void LegParaInit(void)
{
	/*ID号初始化*/
	legOne.ID=LF_LEG;
	
	legTwo.ID=RF_LEG;
	
	legThr.ID=LB_LEG;
	
	legFor.ID=RB_LEG;
	
	legOne.rePos.x=-225.0f;
	legTwo.rePos.x=225.0f;
	legThr.rePos.x=-225.0f;
	legFor.rePos.x=225.0f;
	
	legOne.rePos.y=225.0f;
	legTwo.rePos.y=225.0f;
	legThr.rePos.y=-225.0f;
	legFor.rePos.y=-225.0f;
	
    legFor.ropeTurn=0;
    legOne.ropeTurn=0;
    legTwo.ropeTurn=0;
    legThr.ropeTurn=0;
	
    motor[0].ptCtrl.velLimit  = START_LIMIT;	
	motor[1].ptCtrl.velLimit  = START_LIMIT;	
	motor[2].ptCtrl.velLimit  = START_LIMIT;	
	motor[3].ptCtrl.velLimit  = START_LIMIT;	
    
	/*四足位置初始值*/
	quadrupedStatus.estimatedPos[0]=2950.0f;
	quadrupedStatus.estimatedPos[1]=675.0f;
	
}

/** 
  * @brief  电机初始化配置
  * @note
  * @param  None
  * @retval None
  */
void MoterInit(void)
{
	ElmoInit(CAN1);
	ElmoInit(CAN2);
	PosLoopCfg(CAN2, 1, 1500000000,1500000000,2500000);
	PosLoopCfg(CAN2, 2, 1500000000,1500000000,2500000);
	PosLoopCfg(CAN2, 3, 1500000000,1500000000,2500000);
	PosLoopCfg(CAN2, 4, 1500000000,1500000000,2500000);
	PosLoopCfg(CAN2, LIFT_UP_ID, 900000000,900000000,1500000);

	VelLoopCfg( CAN2 , GYRO1 , 1000000000 , 1000000000 );
  VelLoopCfg( CAN2 , GYRO2 , 1000000000 , 1000000000 );
    
    
	MotorOn(CAN2, 1);
	MotorOn(CAN2, 2);
	MotorOn(CAN2, 3);
	MotorOn(CAN2, 4);
	MotorOn(CAN2, LIFT_UP_ID);
          
    MotorOn( CAN2 , 6 );				//控制电机初始化
	MotorOn( CAN2 , 7 );
    
	//左前腿电机初始化
	VelLoopCfg(CAN1,1,900000000,900000000);
	VelLoopCfg(CAN1,2,900000000,900000000);

	MotorOn(CAN1, 1);
	MotorOn(CAN1, 2);

	//右前腿电机初始化
	VelLoopCfg(CAN1,3,900000000,900000000);
	VelLoopCfg(CAN1,4,900000000,900000000);

	MotorOn(CAN1, 3);
	MotorOn(CAN1, 4);
	
	//左后腿电机初始化
	VelLoopCfg(CAN1,5,900000000,900000000);
	VelLoopCfg(CAN1,6,900000000,900000000);

	MotorOn(CAN1, 5);
	MotorOn(CAN1, 6);
	
	//右后腿电机初始化
	VelLoopCfg(CAN1,7,900000000,900000000);
	VelLoopCfg(CAN1,8,900000000,900000000);
	
	MotorOn(CAN1, 7);
	MotorOn(CAN1, 8);

//	/*腿上电机全部失能*/
//	MotorOff(CAN2, 1);
//	MotorOff(CAN2, 2);
//	MotorOff(CAN2, 3);
//	MotorOff(CAN2, 4);
//	
//	MotorOff(CAN1, 1);
//	MotorOff(CAN1, 2);
//	
//	MotorOff(CAN1, 3);
//	MotorOff(CAN1, 4);
//	
//	MotorOff(CAN1, 5);
//	MotorOff(CAN1, 6);
//	
//	MotorOff(CAN1, 7);
//	MotorOff(CAN1, 8);
//	
//	MotorOff(CAN2, 6);
//	MotorOff(CAN2, 7);
}
/** 
  * @brief  电机失能配置
  * @note
  * @param  None
  * @retval None
  */
void MotorAllOff(void)
{
	MotorOff(CAN2, 1);
	MotorOff(CAN2, 2);
	MotorOff(CAN2, 3);
	MotorOff(CAN2, 4);
	MotorOff(CAN2, LIFT_UP_ID);
	
	MotorOff(CAN1, 1);
	MotorOff(CAN1, 2);
	
	MotorOff(CAN1, 3);
	MotorOff(CAN1, 4);
	
	MotorOff(CAN1, 5);
	MotorOff(CAN1, 6);
	
	MotorOff(CAN1, 7);
	MotorOff(CAN1, 8);

}
void MotorAllStop()
{
    VelCrl(CAN1,legOne.ID,0,0);
    VelCrl(CAN1,legTwo.ID,0,0);
    VelCrl(CAN1,legThr.ID,0,0);
    VelCrl(CAN1,legFor.ID,0,0);

}

/**
* @brief 	准备姿势
* @param    pointAngle：输入点
* @param    motorNum:	电机位置
* @param   	N       插入的点个数
* @param   	LFhip   角度制
* @param   	LFknee  角度制
* @param   	RFhip   角度制
* @param   	RFknee  角度制
* @return   NONE
* @author 	ACTION
* @note     startOver 准备完成后会有产生该标志位，可在主程序里使用
*/

float one_hipInit[N]= {0};
float one_kneeInit[N]={0};	
float two_hipInit[N]={0};
float two_kneeInit[N]={0};
float thr_hipInit[N]= {0};
float thr_kneeInit[N]={0};	
float for_hipInit[N]={0};
float for_kneeInit[N]={0};  
float cp_init=0;
float gyro1InitPos=0;
float gyro2InitPos=0;

float rf_turnInit[N]={0};
float lf_turnInit[N]={0};
float rb_turnInit[N]={0};
float lb_turnInit[N]={0};

void StartInit(float oneHip, float oneKnee, float twoHip, float twoKnee, float thrHip, float thrKnee, float forHip, float forKnee)
{

    
    one_hipInit[0]=two_hipInit[0]=thr_hipInit[0]=for_hipInit[0]=83.0f;          //89.0f;      //
    one_kneeInit[0]=two_kneeInit[0]=thr_kneeInit[0]=for_kneeInit[0]=119.763f;   //131.79f;    //
   
    rf_turnInit[0]=-95.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
    rb_turnInit[0]=-95.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
    
    lf_turnInit[0]=95.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
    lb_turnInit[0]=95.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
   
	gyro1InitPos=gBalancer1.motor.pos;
	gyro2InitPos=gBalancer2.motor.pos;
    
    
    one_hipInit[N-1]=oneHip;
    one_kneeInit[N-1]=oneKnee;
	
	two_hipInit[N-1]=twoHip;
    two_kneeInit[N-1]=twoKnee;
	
	thr_hipInit[N-1]=thrHip;
    thr_kneeInit[N-1]=thrKnee;
	
	for_hipInit[N-1]=forHip;
    for_kneeInit[N-1]=forKnee;
	
	if(site == RED)
	{
        rb_turnInit[N-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
        lf_turnInit[N-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
        lb_turnInit[N-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
        rf_turnInit[N-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
	}
	else if(site == BLUE)
	{
        rb_turnInit[N-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
        lf_turnInit[N-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
        rf_turnInit[N-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
        lb_turnInit[N-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;                
	}
    
	for(uint16_t j=1;j<N;j++)
    {
        one_hipInit[j]=one_hipInit[j-1]-((one_hipInit[0]-one_hipInit[N-1])/(N-1));
        one_kneeInit[j]=one_kneeInit[j-1]-((one_kneeInit[0]-one_kneeInit[N-1])/(N-1));
      
		two_hipInit[j]=two_hipInit[j-1]-((two_hipInit[0]-two_hipInit[N-1])/(N-1));
        two_kneeInit[j]=two_kneeInit[j-1]-((two_kneeInit[0]-two_kneeInit[N-1])/(N-1));
		
		thr_hipInit[j]=thr_hipInit[j-1]-((thr_hipInit[0]-thr_hipInit[N-1])/(N-1));
        thr_kneeInit[j]=thr_kneeInit[j-1]-((thr_kneeInit[0]-thr_kneeInit[N-1])/(N-1));
		
		for_hipInit[j]=for_hipInit[j-1]-((for_hipInit[0]-for_hipInit[N-1])/(N-1));
        for_kneeInit[j]=for_kneeInit[j-1]-((for_kneeInit[0]-for_kneeInit[N-1])/(N-1));
		
        lb_turnInit[j]=lb_turnInit[j-1]-((lb_turnInit[0]-lb_turnInit[N-1])/(N-1));
        rf_turnInit[j]=rf_turnInit[j-1]-((rf_turnInit[0]-rf_turnInit[N-1])/(N-1));
        lf_turnInit[j]=lf_turnInit[j-1]-((lf_turnInit[0]-lf_turnInit[N-1])/(N-1));
        rb_turnInit[j]=rb_turnInit[j-1]-((rb_turnInit[0]-rb_turnInit[N-1])/(N-1));
        
        
    }
        motor[0].ptCtrl.lastPos = transfer(1,83.39f*PI/180.0f,119.79f*PI/180.0f);	
        motor[1].ptCtrl.lastPos = transfer(2,83.39f*PI/180.0f,119.79f*PI/180.0f);	
        motor[2].ptCtrl.lastPos = transfer(3,83.39f*PI/180.0f,119.79f*PI/180.0f);	
        motor[3].ptCtrl.lastPos = transfer(4,83.39f*PI/180.0f,119.79f*PI/180.0f);

    
}



void ReStartInit(float oneHip, float oneKnee, float twoHip, float twoKnee, float thrHip, float thrKnee, float forHip, float forKnee)
{
    one_hipInit[0]=legOne.readPos.hip*180.f/PI;
    two_hipInit[0]=legTwo.readPos.hip*180.f/PI;
    thr_hipInit[0]=legThr.readPos.hip*180.f/PI;
    for_hipInit[0]=legFor.readPos.hip*180.f/PI;          //89.0f;      //
    gyro1InitPos=gBalancer1.motor.pos;
	gyro2InitPos=gBalancer2.motor.pos;

    
    one_kneeInit[0]=legOne.readPos.knee*180.f/PI;
    two_kneeInit[0]=legTwo.readPos.knee*180.f/PI;
    thr_kneeInit[0]=legThr.readPos.knee*180.f/PI;
    for_kneeInit[0]=legFor.readPos.knee*180.f/PI;   //131.79f;    //
   
    lf_turnInit[0]=-legOne.readPos.turn/2.0f/PI*8192.0f*36.0f*48.0f/17.0f;
    rf_turnInit[0]=-legTwo.readPos.turn/2.0f/PI*8192.0f*36.0f*48.0f/17.0f;
    lb_turnInit[0]=-legThr.readPos.turn/2.0f/PI*8192.0f*36.0f*48.0f/17.0f;
    rb_turnInit[0]=-legFor.readPos.turn/2.0f/PI*8192.0f*36.0f*48.0f/17.0f;
    
    one_hipInit[RESETnum-1]=oneHip;
    one_kneeInit[RESETnum-1]=oneKnee;
	
	two_hipInit[RESETnum-1]=twoHip;
    two_kneeInit[RESETnum-1]=twoKnee;
	
	thr_hipInit[RESETnum-1]=thrHip;
    thr_kneeInit[RESETnum-1]=thrKnee;
	
	for_hipInit[RESETnum-1]=forHip;
    for_kneeInit[RESETnum-1]=forKnee;
	
	if(site == RED)
	{
        switch(Reset.status)
        {
            default :
            {
                rf_turnInit[RESETnum-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lb_turnInit[RESETnum-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
                rb_turnInit[RESETnum-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lf_turnInit[RESETnum-1]=-STARTANGLE_RED/360.0f*8192.0f*36.0f*48.0f/17.0f;
                
            }
            break;
            
            case 4 :
            {
                rf_turnInit[RESETnum-1]=90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lf_turnInit[RESETnum-1]=90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f; 
                rb_turnInit[RESETnum-1]=90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lb_turnInit[RESETnum-1]=90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f; 
                
            }
            break;
            case 3 :
            {
                rf_turnInit[RESETnum-1]=-STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lf_turnInit[RESETnum-1]=-STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f; 
                rb_turnInit[RESETnum-1]=-STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lb_turnInit[RESETnum-1]=-STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;                
                
            }
            break;            
        }
	}
	else if(site == BLUE)
	{
        switch(Reset.status)
        {
            default :
            {
                rf_turnInit[RESETnum-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lb_turnInit[RESETnum-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                rb_turnInit[RESETnum-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lf_turnInit[RESETnum-1]=STARTANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
            }
            break;
            case 4 :
            {
                rf_turnInit[RESETnum-1]=-90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lf_turnInit[RESETnum-1]=-90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f; 
                rb_turnInit[RESETnum-1]=-90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lb_turnInit[RESETnum-1]=-90.0f/360.0f*8192.0f*36.0f*48.0f/17.0f;                
            }
            break;
            case 3 :
            {
                rf_turnInit[RESETnum-1]=STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lf_turnInit[RESETnum-1]=STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f; 
                rb_turnInit[RESETnum-1]=STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;
                lb_turnInit[RESETnum-1]=STRANGLE_BLUE/360.0f*8192.0f*36.0f*48.0f/17.0f;                
                
            }
            break;
        }
	}
    
	for(uint16_t j=1;j<RESETnum;j++)
    {
        one_hipInit[j]=one_hipInit[j-1]-((one_hipInit[0]-one_hipInit[RESETnum-1])/(RESETnum-1));
        one_kneeInit[j]=one_kneeInit[j-1]-((one_kneeInit[0]-one_kneeInit[RESETnum-1])/(RESETnum-1));
      
		two_hipInit[j]=two_hipInit[j-1]-((two_hipInit[0]-two_hipInit[RESETnum-1])/(RESETnum-1));
        two_kneeInit[j]=two_kneeInit[j-1]-((two_kneeInit[0]-two_kneeInit[RESETnum-1])/(RESETnum-1));
		
		thr_hipInit[j]=thr_hipInit[j-1]-((thr_hipInit[0]-thr_hipInit[RESETnum-1])/(RESETnum-1));
        thr_kneeInit[j]=thr_kneeInit[j-1]-((thr_kneeInit[0]-thr_kneeInit[RESETnum-1])/(RESETnum-1));
		
		for_hipInit[j]=for_hipInit[j-1]-((for_hipInit[0]-for_hipInit[RESETnum-1])/(RESETnum-1));
        for_kneeInit[j]=for_kneeInit[j-1]-((for_kneeInit[0]-for_kneeInit[RESETnum-1])/(RESETnum-1));
		
        lf_turnInit[j]=lf_turnInit[j-1]-((lf_turnInit[0]-lf_turnInit[RESETnum-1])/(RESETnum-1));
        rf_turnInit[j]=rf_turnInit[j-1]-((rf_turnInit[0]-rf_turnInit[RESETnum-1])/(RESETnum-1));
        lb_turnInit[j]=lb_turnInit[j-1]-((lb_turnInit[0]-lb_turnInit[RESETnum-1])/(RESETnum-1));
        rb_turnInit[j]=rb_turnInit[j-1]-((rb_turnInit[0]-rb_turnInit[RESETnum-1])/(RESETnum-1));
        
    }if(Reset.status==0)
    {
         motor[0].ptCtrl.lastPos = transfer(1,83.39f*PI/180.0f,119.79f*PI/180.0f);	
         motor[1].ptCtrl.lastPos = transfer(2,83.39f*PI/180.0f,119.79f*PI/180.0f);	
         motor[2].ptCtrl.lastPos = transfer(3,83.39f*PI/180.0f,119.79f*PI/180.0f);	
         motor[3].ptCtrl.lastPos = transfer(4,83.39f*PI/180.0f,119.79f*PI/180.0f);
        
        }
   
    
    runningMode=4;
}


int order=0;
extern uint8_t protectFlag;
extern int16_t timeRecord;
extern float CPPos;
uint8_t gvro1ready=0;
uint8_t gvro2ready=0;
float openLoopVel1 = 0.f;
float openLoopVel2 = 0.f;
int openLoopPeriod = 0;  
float openLoopDis1 = 0;
float openLoopDis2 = 0;	
void  PosInit(void)
{   
	legJoint legOnePulse={0},legTwoPulse={0},legThrPulse={0},legForPulse={0};
	if(runningMode==1)
	{
		if(order < N-1)
		{
			legOne.legPos.hip=one_hipInit[order]*PI/180.0f;
			legTwo.legPos.hip=two_hipInit[order]*PI/180.0f;
			legThr.legPos.hip=thr_hipInit[order]*PI/180.0f;
			legFor.legPos.hip=for_hipInit[order]*PI/180.0f;
			
			legOne.legPos.knee=one_kneeInit[order]*PI/180.0f;
			legTwo.legPos.knee=two_kneeInit[order]*PI/180.0f;
			legThr.legPos.knee=thr_kneeInit[order]*PI/180.0f;
			legFor.legPos.knee=for_kneeInit[order]*PI/180.0f;
			
			legOne.legNextPos.hip=one_hipInit[order+1]*PI/180.0f;
			legTwo.legNextPos.hip=two_hipInit[order+1]*PI/180.0f;
			legThr.legNextPos.hip=thr_hipInit[order+1]*PI/180.0f;
			legFor.legNextPos.hip=for_hipInit[order+1]*PI/180.0f;
			
			legOne.legNextPos.knee=one_kneeInit[order+1]*PI/180.0f;
			legTwo.legNextPos.knee=two_kneeInit[order+1]*PI/180.0f;
			legThr.legNextPos.knee=thr_kneeInit[order+1]*PI/180.0f;
			legFor.legNextPos.knee=for_kneeInit[order+1]*PI/180.0f;
			
		}
		else if(order == N-1)
		{
			legOne.legPos.hip=one_hipInit[order]*PI/180.0f;
			legTwo.legPos.hip=two_hipInit[order]*PI/180.0f;
			legThr.legPos.hip=thr_hipInit[order]*PI/180.0f;
			legFor.legPos.hip=for_hipInit[order]*PI/180.0f;
			
			legOne.legPos.knee=one_kneeInit[order]*PI/180.0f;
			legTwo.legPos.knee=two_kneeInit[order]*PI/180.0f;
			legThr.legPos.knee=thr_kneeInit[order]*PI/180.0f;
			legFor.legPos.knee=for_kneeInit[order]*PI/180.0f;
			
			legOne.legNextPos.hip=one_hipInit[order]*PI/180.0f;
			legTwo.legNextPos.hip=two_hipInit[order]*PI/180.0f;
			legThr.legNextPos.hip=thr_hipInit[order]*PI/180.0f;
			legFor.legNextPos.hip=for_hipInit[order]*PI/180.0f;
				   
			legOne.legNextPos.knee=one_kneeInit[order]*PI/180.0f;
			legTwo.legNextPos.knee=two_kneeInit[order]*PI/180.0f;
			legThr.legNextPos.knee=thr_kneeInit[order]*PI/180.0f;
			legFor.legNextPos.knee=for_kneeInit[order]*PI/180.0f;
		
		}
	}
	else if(runningMode==4)
	{
	
		if(order < RESETnum-1)
		{
			legOne.legPos.hip=one_hipInit[order]*PI/180.0f;
			legTwo.legPos.hip=two_hipInit[order]*PI/180.0f;
			legThr.legPos.hip=thr_hipInit[order]*PI/180.0f;
			legFor.legPos.hip=for_hipInit[order]*PI/180.0f;
			
			legOne.legPos.knee=one_kneeInit[order]*PI/180.0f;
			legTwo.legPos.knee=two_kneeInit[order]*PI/180.0f;
			legThr.legPos.knee=thr_kneeInit[order]*PI/180.0f;
			legFor.legPos.knee=for_kneeInit[order]*PI/180.0f;
			
			legOne.legNextPos.hip=one_hipInit[order+1]*PI/180.0f;
			legTwo.legNextPos.hip=two_hipInit[order+1]*PI/180.0f;
			legThr.legNextPos.hip=thr_hipInit[order+1]*PI/180.0f;
			legFor.legNextPos.hip=for_hipInit[order+1]*PI/180.0f;
			
			legOne.legNextPos.knee=one_kneeInit[order+1]*PI/180.0f;
			legTwo.legNextPos.knee=two_kneeInit[order+1]*PI/180.0f;
			legThr.legNextPos.knee=thr_kneeInit[order+1]*PI/180.0f;
			legFor.legNextPos.knee=for_kneeInit[order+1]*PI/180.0f;
			
		}
		else if(order == RESETnum-1)
		{
			legOne.legPos.hip=one_hipInit[order]*PI/180.0f;
			legTwo.legPos.hip=two_hipInit[order]*PI/180.0f;
			legThr.legPos.hip=thr_hipInit[order]*PI/180.0f;
			legFor.legPos.hip=for_hipInit[order]*PI/180.0f;
			
			legOne.legPos.knee=one_kneeInit[order]*PI/180.0f;
			legTwo.legPos.knee=two_kneeInit[order]*PI/180.0f;
			legThr.legPos.knee=thr_kneeInit[order]*PI/180.0f;
			legFor.legPos.knee=for_kneeInit[order]*PI/180.0f;
			
			legOne.legNextPos.hip=one_hipInit[order]*PI/180.0f;
			legTwo.legNextPos.hip=two_hipInit[order]*PI/180.0f;
			legThr.legNextPos.hip=thr_hipInit[order]*PI/180.0f;
			legFor.legNextPos.hip=for_hipInit[order]*PI/180.0f;
				   
			legOne.legNextPos.knee=one_kneeInit[order]*PI/180.0f;
			legTwo.legNextPos.knee=two_kneeInit[order]*PI/180.0f;
			legThr.legNextPos.knee=thr_kneeInit[order]*PI/180.0f;
			legFor.legNextPos.knee=for_kneeInit[order]*PI/180.0f;
		
		}
	}
	
	/*航向角控制*/
	PosCrl2(CAN2,1,ABSOLUTE_MODE,lf_turnInit[order]);
	PosCrl2(CAN2,2,ABSOLUTE_MODE,rf_turnInit[order]);
	PosCrl2(CAN2,3,ABSOLUTE_MODE,lb_turnInit[order]);
	PosCrl2(CAN2,4,ABSOLUTE_MODE,rb_turnInit[order]); 
	
	order++;
    
   if(Reset.status==4)
   {
        if(site == RED)
        {
            gvro1ready = GoToOpenLoopInitPos1(40.0f, 0.f);
            gvro2ready = GoToOpenLoopInitPos2(40.0f, 0.f);
        }
        else   
        {
            gvro1ready = GoToOpenLoopInitPos1(55.0f, 0.f);
            gvro2ready = GoToOpenLoopInitPos2(25.0f, 0.f);
        }
    }
    else if(Reset.status==1||Reset.status==0)
    {

        if(site == RED)
        {
            gvro1ready = GoToOpenLoopInitPos1(ZERO_ENCODER_POS+25.0f, openLoopDis1);
            gvro2ready = GoToOpenLoopInitPos2(ZERO_ENCODER_POS+25.0f, openLoopDis2);
        }
        else
        {
            gvro1ready = GoToOpenLoopInitPos1(ZERO_ENCODER_POS+25.0f, openLoopDis1);
            gvro2ready = GoToOpenLoopInitPos2(ZERO_ENCODER_POS+25.0f, openLoopDis2);
        }
    }
    else if(Reset.status==3)
    {

        if(site == RED)
        {
            gvro1ready = GoToOpenLoopInitPos1(ZERO_ENCODER_POS + 25.0f, openLoopDis1);
            gvro2ready = GoToOpenLoopInitPos2(ZERO_ENCODER_POS +5.0f, openLoopDis2);
        }
        else
        {
            gvro1ready = GoToOpenLoopInitPos1(ZERO_ENCODER_POS+5.0f, openLoopDis1);
            gvro2ready = GoToOpenLoopInitPos2(ZERO_ENCODER_POS+25.0f,  openLoopDis2);
        }
    }




	
    if(runningMode==1)
	{
		legOnePulse=transfer(1,one_hipInit[N-1]*PI/180.0f,one_kneeInit[N-1]*PI/180.0f);
		legTwoPulse=transfer(3,two_hipInit[N-1]*PI/180.0f,two_kneeInit[N-1]*PI/180.0f);
		legThrPulse=transfer(5,thr_hipInit[N-1]*PI/180.0f,thr_kneeInit[N-1]*PI/180.0f);
		legForPulse=transfer(7,for_hipInit[N-1]*PI/180.0f,for_kneeInit[N-1]*PI/180.0f);
		
        if(order>=N) 
            order--; 
        if((fabs(fabs(motor[0].posCtrl.actualPos.hip) -fabs(legOnePulse.hip))<500  &&\
            fabs(fabs(motor[0].posCtrl.actualPos.knee)-fabs(legOnePulse.knee))<500 &&\
            fabs(fabs(motor[1].posCtrl.actualPos.hip) -fabs(legTwoPulse.hip))<500  &&\
            fabs(fabs(motor[1].posCtrl.actualPos.knee)-fabs(legTwoPulse.knee))<500 &&\
            fabs(fabs(motor[2].posCtrl.actualPos.hip) -fabs(legThrPulse.hip))<500  &&\
            fabs(fabs(motor[2].posCtrl.actualPos.knee)-fabs(legThrPulse.knee))<500 &&\
            fabs(fabs(motor[3].posCtrl.actualPos.hip) -fabs(legForPulse.hip))<500 &&\
            fabs(fabs(motor[3].posCtrl.actualPos.knee)-fabs(legForPulse.knee))<500)&&\
            (order>=(N-1)))	
			{	  
				correctCnt++;
			}
        else 
            correctCnt=0;
    }
    if(runningMode==4)
    {
		legOnePulse=transfer(1,one_hipInit[RESETnum-1]*PI/180.0f,one_kneeInit[RESETnum-1]*PI/180.0f);
		legTwoPulse=transfer(3,two_hipInit[RESETnum-1]*PI/180.0f,two_kneeInit[RESETnum-1]*PI/180.0f);
		legThrPulse=transfer(5,thr_hipInit[RESETnum-1]*PI/180.0f,thr_kneeInit[RESETnum-1]*PI/180.0f);
		legForPulse=transfer(7,for_hipInit[RESETnum-1]*PI/180.0f,for_kneeInit[RESETnum-1]*PI/180.0f);
		
        if(order>=RESETnum) 
            order--; 
        if((fabs(fabs(motor[0].posCtrl.actualPos.hip) -fabs(legOnePulse.hip))<500  &&\
            fabs(fabs(motor[0].posCtrl.actualPos.knee)-fabs(legOnePulse.knee))<500 &&\
            fabs(fabs(motor[1].posCtrl.actualPos.hip) -fabs(legTwoPulse.hip))<500  &&\
            fabs(fabs(motor[1].posCtrl.actualPos.knee)-fabs(legTwoPulse.knee))<500 &&\
            fabs(fabs(motor[2].posCtrl.actualPos.hip) -fabs(legThrPulse.hip))<500  &&\
            fabs(fabs(motor[2].posCtrl.actualPos.knee)-fabs(legThrPulse.knee))<500 &&\
            fabs(fabs(motor[3].posCtrl.actualPos.hip) -fabs(legForPulse.hip))<500 &&\
            fabs(fabs(motor[3].posCtrl.actualPos.knee)-fabs(legForPulse.knee))<500)&&\
            (order>=(RESETnum-1)))	
			{	  
				correctCnt++;
			}
        else 
            correctCnt=0;        
    }
    
    
    
	if(correctCnt >= 100 && gvro1ready==1&&gvro2ready==1) 
	{
        if(runningMode==1)
            startOver=1;
        else 
		{
			startOver=1;
			runningMode=5;
			
		}
		correctCnt=0;
	}
	
	/*肩、膝关节电机控制*/
	VelOutput(legOne.legPos.hip,legOne.legNextPos.hip,legOne.legPos.knee,legOne.legNextPos.knee,legOne.ID);
	VelOutput(legTwo.legPos.hip,legTwo.legNextPos.hip,legTwo.legPos.knee,legTwo.legNextPos.knee,legTwo.ID);
	VelOutput(legThr.legPos.hip,legThr.legNextPos.hip,legThr.legPos.knee,legThr.legNextPos.knee,legThr.ID);
	VelOutput(legFor.legPos.hip,legFor.legNextPos.hip,legFor.legPos.knee,legFor.legNextPos.knee,legFor.ID);
	
}



