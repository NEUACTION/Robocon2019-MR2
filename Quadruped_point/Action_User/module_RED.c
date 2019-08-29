#include "module_RED.h"
#include "vloop.h"
#include "init.h"
#include "distanceAdjust.h"
#include "balance.h"
#include "point.h"
#include "dma.h"
#include "pid.h"
#include "switch.h"
#include "pps.h"
#include "stm32f4xx_it.h"
/*上坡步态，0为240，1为新240，2为250*/
#define STEP_S (0)
/*********全局变量*************/
/*步数*/
uint16_t gaitCnt=0;

/*期望角度*/
float desireAngle_X=0,desireAngle_Y=0;
/***********extern*************/
extern leg_ legOne, legTwo, legThr, legFor;  
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern visual_ visual;
extern uint8_t site;
extern uint8_t adjustFlag;
extern BalancerType gBalancer1;
extern BalancerType gBalancer2;
extern uint16_t DebugUSARTSendBuffCnt;
extern uint8_t DebugUSARTSendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern uint8_t DebugUSARTDMASendBuf[DEBUG_USART_SEND_BUF_CAPACITY];
extern gyro_t lf_rb,rf_lb;
extern float openLoopVel1,openLoopVel2;
extern float closeLoopForward_y ,closeLoopForward_x ;
extern uint8_t goBack_flag_y, goBack_flag_x,goBack_pos_y, goBack_pos_x;
extern Serial_ serial1;
extern Serial_ serial2;

/*起步肩关节摆动相*/
extern float hip_start_swing[26];
	
/*起步肩关节支撑相*/
extern float hip_start_support[26];

/*起步膝关节摆动相*/
extern float knee_start_swing[26];

/*起步膝关节支撑相*/
extern float knee_start_support[26];

/*平地直线行走步态肩关节摆动相*/
extern float stright_hip_swing[26];

/*平地直线行走步态肩关节支撑相*/
extern float stright_hip_support[26];

/*平地直线行走步态膝关节摆动相*/
extern float stright_knee_swing[26];

/*平地直线行走步态膝关节支撑相*/
extern float stright_knee_support[26];

/*303转335平地直线行走步态肩关节摆动相*/
extern float stright_hip_swing_1[26];

/*303转335平地直线行走步态肩关节摆动相*/
extern float stright_hip_support_1[26];

/*303转335平地直线行走步态肩关节摆动相*/
extern  float stright_knee_swing_1[26];

/*303转335平地直线行走步态肩关节摆动相*/
extern float stright_knee_support_1[26];

/*335转370平地直线行走步态肩关节摆动相*/
extern float stright_hip_swing_2[26];

/*335转370平地直线行走步态肩关节摆动相*/
extern float stright_hip_support_2[26];

/*335转370平地直线行走步态肩关节摆动相*/
extern float stright_knee_swing_2[26];

/*335转370平地直线行走步态肩关节摆动相*/
extern float stright_knee_support_2[26];

/*上台阶步态过渡肩关节摆动相*/
extern float stepup_hip_swing_trans[26];

/*上台阶步态过渡膝关节摆动相*/
extern float stepup_knee_swing_trans[26];

/*平地450到480步态过渡肩关节摆动相*/
extern float step_hip_swing_trans[26];

/*平地450到480步态过渡膝关节摆动相*/
extern float step_knee_swing_trans[26];

/*平地450到480步态过渡肩关节支撑*/
extern float step_hip_support_trans[26];

/*平地450到480步态过渡膝关节摆动相*/
extern float step_knee_support_trans[26];

/*过台步态肩关节正常摆动相*/
extern float step_hip_swing[26];

/*过台步态肩关节正常摆动相*/
extern float step_knee_swing[26];

/*过台步态肩关节正常支撑相*/
extern float step_hip_support[26];

/*过台步态肩关节正常支撑相*/
extern float step_knee_support[26];

/*上台阶步态肩关节摆动相*/
extern float stepup_hip_swing[26];

/*上台阶步态膝关节摆动相*/
extern float stepup_knee_swing[26];

/*下台阶步态肩关节摆动相*/
extern float stepdown_hip_swing[26];

/*下台阶步态膝关节摆动相*/
extern float stepdown_knee_swing[26];

/*台阶上支撑相步态肩关节*/
extern float onStep_hip_support[26];

/*台阶上支撑相步态膝关节*/
extern float onStep_knee_support[26];

/*台阶上支撑相步态肩关节*/
extern float onStep_hip_support_1[26];

/*台阶上支撑相步态膝关节*/
extern float onStep_knee_support_1[26];

/*下台阶步态肩关节摆动相*/
extern float stepdown_hip_swing_1[26];

/*下台阶步态膝关节摆动相*/
extern float stepdown_knee_swing_1[26];

/*上台阶步态肩关节摆动相*/
extern float stepup_hip_swing_2[26];

/*上台阶步态膝关节摆动相*/
extern float stepup_knee_swing_2[26];

/*台阶上支撑相步态肩关节*/
extern float onStep_hip_support_2[26];

/*台阶上支撑相步态膝关节*/
extern float onStep_knee_support_2[26];

/*台阶上支撑相步态肩关节*/
extern float stright_hip_support_4[26];

/*台阶上支撑相步态膝关节*/
extern float stright_knee_support_4[26];

/*下台阶步态肩关节摆动相*/
extern float stepup_hip_swing_4[26];

/*下台阶步态膝关节摆动相*/
extern float stepup_knee_swing_4[26];

/*下台到绳前肩关节摆动相过渡*/
extern float STR_hip_swing_trans_1[26];

/*下台到绳前膝关节摆动相过渡*/
extern float STR_knee_swing_trans_1[26];

/*下台到绳前肩关节支撑相过渡*/
extern float STR_hip_support_trans_1[26];

/*下台到绳前膝关节支撑相过渡*/
extern float STR_knee_support_trans_1[26];

/*下台到绳前肩关节摆动相*/
extern float STR_hip_swing[26];

/*下台到绳前膝关节摆动相*/
extern float STR_knee_swing[26];

/*下台到绳前肩关节摆动相*/
extern float STR_hip_support[26];

/*下台到绳前膝关节摆动相*/
extern float STR_knee_support[26];

/*下台到绳前肩关节摆动相过渡*/
extern float STR_hip_swing_trans_2[26];

/*下台到绳前膝关节摆动相过渡*/
extern float STR_knee_swing_trans_2[26];

/*下台到绳前肩关节支撑相过渡*/
extern float STR_hip_support_trans_2[26];

/*下台到绳前膝关节支撑相过渡*/
extern float STR_knee_support_trans_2[26];

/*过绳步态肩关节先低后高过渡*/	
extern float rope_hip_trans_LH[26];

/*过绳步态膝关节先低后高过渡*/	
extern float rope_knee_trans_LH[26];

/*过绳步态膝关节支撑相过渡*/	
extern float rope_hip_trans_support[26];

/*过绳步态膝关节支撑相过渡*/	
extern float rope_knee_trans_support[26];

/*过绳步态膝关节支撑相过渡*/	
extern float rope_hip_trans_stright[26];

/*过绳步态膝关节支撑相过渡*/	
extern float rope_knee_trans_stright[26];

/*过绳步态肩关节支撑相*/	
extern float rope_hip_swing[26];

/*过绳步态膝关节支撑相*/	
extern float rope_knee_swing[26];

/*过绳步态肩关节支撑想*/	
extern float rope_hip_support[26];

/*过绳步态膝关节支撑想*/	
extern float rope_knee_support[26];

/*过绳步态肩关节摆动相先高后低*/	
extern float rope_hip_HL_swing[26];

/*过绳步态膝关节摆动相先高后低*/	
extern float rope_knee_HL_swing[26];

/*过绳步态肩关节摆动相先低后高*/	
extern float rope_hip_LH_swing[26];

/*过绳步态膝关节摆动相先低后高*/	
extern float rope_knee_LH_swing[26];

/*450过渡到250肩关节摆动相*/
extern float slope_hip_trans_swing[26];

/*450过渡到250膝关节摆动相*/
extern float slope_knee_trans_swing[26];

/*450过渡到250肩关节支撑相*/
extern float slope_hip_trans_support[26];

/*450过渡到250膝关节支撑相*/
extern float slope_knee_trans_support[26];

/*平面到坡过渡步态左前腿肩关节第1步*/	
extern float PTS_one_hip_1[21];

/*平面到坡过渡步态左前腿肩关节第2步*/	
extern float PTS_one_hip_2[21];

/*平面到坡过渡步态左前腿肩关节第3步*/	
extern float PTS_one_hip_3[21];

/*平面到坡过渡步态左前腿肩关节第4步*/	
extern float PTS_one_hip_4[21];

/*平面到坡过渡步态左前腿膝关节第1步*/	
extern float PTS_one_knee_1[21];

/*平面到坡过渡步态左前腿膝关节第2步*/	
extern float PTS_one_knee_2[21];

/*平面到坡过渡步态左前腿膝关节第3步*/	
extern float PTS_one_knee_3[21];

/*平面到坡过渡步态左前腿膝关节第4步*/	
extern float PTS_one_knee_4[21];

/*平面到坡过渡步态2号腿肩关节第1步*/	
extern float PTS_two_hip_1[21];

/*平面到坡过渡步态2号腿肩关节第2步*/	
extern float PTS_two_hip_2[21];

/*平面到坡过渡步态2号腿肩关节第3步*/	
extern float PTS_two_hip_3[21];

/*平面到坡过渡步态2号腿肩关节第4步*/	
extern float PTS_two_hip_4[21];

/*平面到坡过渡步态2号腿膝关节第1步*/	
extern float PTS_two_knee_1[21];

/*平面到坡过渡步态2号腿膝关节第2步*/	
extern float PTS_two_knee_2[21];

/*平面到坡过渡步态2号腿膝关节第3步*/	
extern float PTS_two_knee_3[21];

/*平面到坡过渡步态2号腿膝关节第4步*/	
extern float PTS_two_knee_4[21];

/*平面到坡过渡步态3号腿肩关节第1步*/	
extern float PTS_thr_hip_1[21];

/*平面到坡过渡步态3号腿肩关节第2步*/	
extern float PTS_thr_hip_2[21];

/*平面到坡过渡步态3号腿肩关节第3步*/	
extern float PTS_thr_hip_3[21];

/*平面到坡过渡步态3号腿肩关节第4步*/	
extern float PTS_thr_hip_4[21];

/*平面到坡过渡步态左后腿膝关节第1步*/	
extern float PTS_thr_knee_1[21];

/*平面到坡过渡步态左后腿膝关节第2步*/	
extern float PTS_thr_knee_2[21];

/*平面到坡过渡步态左后腿膝关节第3步*/	
extern float PTS_thr_knee_3[21];

/*平面到坡过渡步态左后腿膝关节第4步*/	
extern float PTS_thr_knee_4[21];

/*平面到坡过渡步态右后腿肩关节第1步*/	
extern float PTS_for_hip_1[21];

/*平面到坡过渡步态右后腿肩关节第2步*/	
extern float PTS_for_hip_2[21];

/*平面到坡过渡步态右后腿肩关节第3步*/	
extern float PTS_for_hip_3[21];

/*平面到坡过渡步态右后腿肩关节第4步*/	
extern float PTS_for_hip_4[21];

/*平面到坡过渡步态右后腿膝关节第1步*/	
extern float PTS_for_knee_1[21];

/*平面到坡过渡步态右后腿膝关节第2步*/	
extern float PTS_for_knee_2[21];

/*平面到坡过渡步态右后腿膝关节第3步*/	
extern float PTS_for_knee_3[21];

/*平面到坡过渡步态右后腿膝关节第4步*/	
extern float PTS_for_knee_4[21];

/*上坡步态肩关节摆动相*/	
extern float slope_hip_swing[21];

/*上坡步态肩关节支撑相*/	
extern float slope_hip_support[21];

/*上坡步态膝关节摆动相*/	
extern float slope_knee_swing[21];

/*上坡步态膝关节支撑相*/	
extern float slope_knee_support[21];

/*坡到平面过渡步态1号腿肩关节第1步*/	
extern float STP_one_hip_1[21];

/*坡到平面过渡步态1号腿肩关节第2步*/	
extern float STP_one_hip_2[21];

/*坡到平面过渡步态1号腿肩关节第3步*/	
extern float STP_one_hip_3[21];

/*坡到平面过渡步态1号腿肩关节第4步*/	
extern float STP_one_hip_4[21];

/*坡到平面过渡步态1号腿膝关节第1步*/	
extern float STP_one_knee_1[21];

/*坡到平面过渡步态1号腿膝关节第2步*/	
extern float STP_one_knee_2[21];

/*坡到平面过渡步态1号腿膝关节第3步*/	
extern float STP_one_knee_3[21];

/*坡到平面过渡步态1号腿膝关节第4步*/	
extern float STP_one_knee_4[21];

/*坡到平面过渡步态2号腿肩关节第1步*/	
extern float STP_two_hip_1[21];

/*坡到平面过渡步态2号腿肩关节第2步*/	
extern float STP_two_hip_2[21];

/*坡到平面过渡步态2号腿肩关节第3步*/	
extern float STP_two_hip_3[21];

/*坡到平面过渡步态2号腿肩关节第4步*/	
extern float STP_two_hip_4[21];

/*坡到平面过渡步态2号腿膝关节第1步*/	
extern float STP_two_knee_1[21];

/*坡到平面过渡步态2号腿膝关节第2步*/	
extern float STP_two_knee_2[21];

/*坡到平面过渡步态2号腿膝关节第3步*/	
extern float STP_two_knee_3[21];

/*坡到平面过渡步态2号腿膝关节第4步*/	
extern float STP_two_knee_4[21];

/*坡到平面过渡步态3号腿肩关节第1步*/	
extern float STP_thr_hip_1[21];

/*坡到平面过渡步态3号腿肩关节第2步*/	
extern float STP_thr_hip_2[21];

/*坡到平面过渡步态3号腿肩关节第3步*/	
extern float STP_thr_hip_3[21];

/*坡到平面过渡步态3号腿肩关节第4步*/	
extern float STP_thr_hip_4[21];

/*坡到平面过渡步态左后腿膝关节第1步*/	
extern float STP_thr_knee_1[21];

/*坡到平面过渡步态左后腿膝关节第2步*/	
extern float STP_thr_knee_2[21];

/*坡到平面过渡步态左后腿膝关节第3步*/	
extern float STP_thr_knee_3[21];

/*坡到平面过渡步态左后腿膝关节第4步*/	
extern float STP_thr_knee_4[21];

/*坡到平面过渡步态右后腿肩关节第1步*/	
extern float STP_for_hip_1[21];

/*平面到坡过渡步态右后腿肩关节第2步*/	
extern float STP_for_hip_2[21];

/*平面到坡过渡步态右后腿肩关节第3步*/	
extern float STP_for_hip_3[21];

/*平面到坡过渡步态右后腿肩关节第4步*/	
extern float STP_for_hip_4[21];

/*坡到平面过渡步态右后腿膝关节第1步*/	
extern float STP_for_knee_1[21];

/*坡到平面过渡步态右后腿膝关节第2步*/	
extern float STP_for_knee_2[21];

/*坡到平面过渡步态右后腿膝关节第2步*/	
extern float STP_for_knee_3[21];

/*坡到平面过渡步态右后腿膝关节第4步*/	
extern float STP_for_knee_4[21];

/** 
  * @brief  起步步态
  * @note
  * @param  None
  * @retval None
  */
  
uint8_t startCnt=0;    
uint8_t strightStep=0;
uint8_t strightCnt=0;
uint8_t stepStep=0,stepCnt=0;    
uint8_t STRStep=0;
uint8_t STRCnt=0;    
uint8_t ropeStep=0;
uint8_t ropeCnt=0;    
uint8_t relayCnt=0;    
uint8_t hillStep=0;
uint8_t hillCnt=0;
  
uint8_t StartUp_RED(void)
{
    
	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;

    if(startCnt > 5)
    {
        closeLoopForward_x=-250.0f;
        
    }
    quadrupedControl.legAngle=STARTANGLE_RED*PI/180.0f;
    
	legOne.standFlag=legFor.standFlag=1;
	legTwo.standFlag=legThr.standFlag=0;
	
	/*当前想到达的位置*/
	one_hip =hip_start_support[startCnt];
	one_knee=knee_start_support[startCnt];
	
	two_hip =hip_start_swing[startCnt];
	two_knee=knee_start_swing[startCnt];
	
	thr_hip =hip_start_swing[startCnt];
	thr_knee=knee_start_swing[startCnt];
	
	for_hip =hip_start_support[startCnt];
	for_knee=knee_start_support[startCnt];
	
	/*下一个想到达的位置*/
	one_hip_next =hip_start_support[startCnt+1];
	one_knee_next=knee_start_support[startCnt+1];
	
	two_hip_next =hip_start_swing[startCnt+1];
	two_knee_next=knee_start_swing[startCnt+1];
	
	thr_hip_next =hip_start_swing[startCnt+1];
	thr_knee_next=knee_start_swing[startCnt+1];
	
	for_hip_next =hip_start_support[startCnt+1];
	for_knee_next=knee_start_support[startCnt+1];
	
	
	startCnt++;

	TurnMotorControl();
	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);

	if(startCnt >= 25)
	{
		gaitCnt++;
		return 1;
	}
	else
		return 0;
}

/** 
  * @brief  走直线
  * @note
  * @param  None
  * @retval None
  */

	 
uint8_t Stright_RED(void) 
{
	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;
	
	quadrupedControl.legAngle=STARTANGLE_RED*PI/180.0f;
	
    if(gaitCnt == 1)
	{

		goBack_flag_y=0;
		closeLoopForward_x=0.0f;

       	if(strightCnt >= 6)
        {
            closeLoopForward_y=-400.0f;
        }
		if(strightCnt >= 6)
		{
			goBack_pos_x = 30.f;
			goBack_flag_x = 1;
		}
	}
    else if(gaitCnt == 2)
	{	
		goBack_flag_x=0;
        closeLoopForward_y=0.0f;
		if(strightCnt <= 13)
			closeLoopForward_x=150.0f;
        else 
            closeLoopForward_x=-350.0f;
		if(strightCnt >= 6)
		{
			goBack_pos_y = 30.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 3)
	{
		goBack_flag_y=0;
		closeLoopForward_x=0.0f;
        if(strightCnt<=14)
            closeLoopForward_y=350.0f;
		else
            closeLoopForward_y=-180.0f;
		
		if(strightCnt >= 6)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
    else if(gaitCnt == 4)
	{	
		goBack_flag_x=0;
		closeLoopForward_y=0.0f;
        if(strightCnt<=13)
            closeLoopForward_x=50.0f;
		else 
			closeLoopForward_x=-100.0f;
		if(strightCnt >= 6)
		{
			goBack_pos_y = 30.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 5)
	{
		goBack_flag_y=0;
		closeLoopForward_x=0.0f;
        if(strightCnt<=13)
            closeLoopForward_y=200.0f;
		else 
            closeLoopForward_y=-250.0f;
		if(strightCnt >= 6)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
    else if(gaitCnt == 6)
	{	
		goBack_flag_x=0;
		closeLoopForward_y=0.0f;
        if(strightCnt<14)
            closeLoopForward_x=100.0f;
        else closeLoopForward_x=50.0f;
		if(strightCnt >= 6)
		{
			goBack_pos_y = 40.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 7)
	{
		goBack_flag_y=0;
		closeLoopForward_x=0.0f;
        if(strightCnt<= 10)
            closeLoopForward_y=170.0f;
		else 
            closeLoopForward_y=-150.0f;
		
		if(strightCnt >= 6)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
    else if(gaitCnt == 8)
	{	
		goBack_flag_x=0;
		closeLoopForward_y=0.0f;
        
        if(strightCnt<=15)
            closeLoopForward_x=100.0f;
        else closeLoopForward_x =-50;
        
		if(strightCnt >= 6)
		{
			goBack_pos_y = 40.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 9)
	{
		goBack_flag_y=0;
		closeLoopForward_x=0.0f;
        
        if(strightCnt<=15)
            closeLoopForward_y=160.0f;
		else 
			closeLoopForward_y=-150.0f;
		
        if(strightCnt >= 6)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
    else if(gaitCnt == 10)
	{	
		goBack_flag_x=0;
		closeLoopForward_y=0.0f;
       if(strightCnt>=15)
            closeLoopForward_x=-20.0f;
        else closeLoopForward_x=-120.0f;
		if(strightCnt >= 6)
		{
			goBack_pos_y = 40.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 11)
	{
		goBack_flag_y=0;
		closeLoopForward_x=0.0f;
        
        if(strightCnt<=13)
            closeLoopForward_y=100.0f;
		else 
			closeLoopForward_y=-280.0f ;
		
		if(strightCnt >= 6)
		{
			goBack_pos_x = 15.f;
			goBack_flag_x = 1;
		}
	}
    else if(gaitCnt == 12)
	{	
		goBack_flag_x=0;
		closeLoopForward_y=0.0f;
        
        if(strightCnt<=20)
            closeLoopForward_x=270.0f;
        else closeLoopForward_x=150.0f;
        
		if(strightCnt >= 6)
		{
			goBack_pos_y = 40.f;
			goBack_flag_y = 1;
		}
	}
    
	switch(strightStep)
	{
		//左前右后支撑
		case 0:
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;
			
			/*当前想到达的位置*/
			one_hip =stright_hip_swing[strightCnt];
			one_knee=stright_knee_swing[strightCnt];
			
			two_hip =stright_hip_support[strightCnt];
			two_knee=stright_knee_support[strightCnt];
		    
			thr_hip =stright_hip_support[strightCnt];
			thr_knee=stright_knee_support[strightCnt];
		    
			for_hip =stright_hip_swing[strightCnt];
			for_knee=stright_knee_swing[strightCnt];
		
			/*下一个想到达的位置*/
			one_hip_next =stright_hip_swing[strightCnt+1];
		    one_knee_next=stright_knee_swing[strightCnt+1];
		    
		    two_hip_next =stright_hip_support[strightCnt+1];
		    two_knee_next=stright_knee_support[strightCnt+1];
		    
		    thr_hip_next =stright_hip_support[strightCnt+1];
		    thr_knee_next=stright_knee_support[strightCnt+1];
		    
		    for_hip_next =stright_hip_swing[strightCnt+1];
		    for_knee_next=stright_knee_swing[strightCnt+1];

            gBalancer1.cnt=strightCnt;
			strightCnt++;
			if(strightCnt >= 25)
			{
				strightStep++;
				strightCnt=0;
				gaitCnt++;
			}
			break;
		//右前左后支撑
		case 1:
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;
			
			/*当前想到达的位置*/
			one_hip =stright_hip_support[strightCnt];
			one_knee=stright_knee_support[strightCnt];
		    
			two_hip =stright_hip_swing[strightCnt];
			two_knee=stright_knee_swing[strightCnt];
		    
			thr_hip =stright_hip_swing[strightCnt];
			thr_knee=stright_knee_swing[strightCnt];
		    
			for_hip =stright_hip_support[strightCnt];
			for_knee=stright_knee_support[strightCnt];
		
			/*下一个想到达的位置*/
			one_hip_next =stright_hip_support[strightCnt+1];
			one_knee_next=stright_knee_support[strightCnt+1];
		    
			two_hip_next =stright_hip_swing[strightCnt+1];
			two_knee_next=stright_knee_swing[strightCnt+1];
		    
			thr_hip_next =stright_hip_swing[strightCnt+1];
			thr_knee_next=stright_knee_swing[strightCnt+1];
		    
			for_hip_next =stright_hip_support[strightCnt+1];
			for_knee_next=stright_knee_support[strightCnt+1];
			
            gBalancer1.cnt=strightCnt;
			strightCnt++;
			if(strightCnt >= 25)
			{
				
				strightCnt=0;
				gaitCnt++;
				if(gaitCnt == 11)
					strightStep=2;
				else
					strightStep=0;
			}
			break;
		case 2:
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;
		
			/*当前想到达的位置*/
			one_hip =stright_hip_swing_1[strightCnt];
			one_knee=stright_knee_swing_1[strightCnt];
			
			two_hip =stright_hip_support_1[strightCnt];
			two_knee=stright_knee_support_1[strightCnt];
		    
			thr_hip =stright_hip_support_1[strightCnt];
			thr_knee=stright_knee_support_1[strightCnt];
		    
			for_hip =stright_hip_swing_1[strightCnt];
			for_knee=stright_knee_swing_1[strightCnt];
		
			/*下一个想到达的位置*/
			one_hip_next =stright_hip_swing_1[strightCnt+1];
			one_knee_next=stright_knee_swing_1[strightCnt+1];
		    
		    two_hip_next =stright_hip_support_1[strightCnt+1];
		    two_knee_next=stright_knee_support_1[strightCnt+1];
		    
		    thr_hip_next =stright_hip_support_1[strightCnt+1];
		    thr_knee_next=stright_knee_support_1[strightCnt+1];
		    
		    for_hip_next =stright_hip_swing_1[strightCnt+1];
		    for_knee_next=stright_knee_swing_1[strightCnt+1];
			
            gBalancer1.cnt=strightCnt;
			strightCnt++;
			if(strightCnt >= 25)
			{
				strightStep++;
				strightCnt=0;
				gaitCnt++;
			}
			break;
		//右前左后支撑
		case 3:
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;
			
			/*当前想到达的位置*/
			one_hip =stright_hip_support_2[strightCnt];
			one_knee=stright_knee_support_2[strightCnt];
		    
			two_hip =stright_hip_swing_2[strightCnt];
			two_knee=stright_knee_swing_2[strightCnt];
		    
			thr_hip =stright_hip_swing_2[strightCnt];
			thr_knee=stright_knee_swing_2[strightCnt];
		    
			for_hip =stright_hip_support_2[strightCnt];
			for_knee=stright_knee_support_2[strightCnt];
			
			/*下一个想到达的位置*/
			one_hip_next =stright_hip_support_2[strightCnt+1];
			one_knee_next=stright_knee_support_2[strightCnt+1];
		    
			two_hip_next =stright_hip_swing_2[strightCnt+1];
			two_knee_next=stright_knee_swing_2[strightCnt+1];
		    
			thr_hip_next =stright_hip_swing_2[strightCnt+1];
			thr_knee_next=stright_knee_swing_2[strightCnt+1];
		    
			for_hip_next =stright_hip_support_2[strightCnt+1];
			for_knee_next=stright_knee_support_2[strightCnt+1];
				
            gBalancer1.cnt=strightCnt;
			strightCnt++;
			if(strightCnt >= 25)
			{
				strightStep=0;
				strightCnt=0;
				gaitCnt++;
			}
			break;
		default:
			break;
			
	}

	TurnMotorControl();
	
	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);
	
	if(gaitCnt >= 13)
		return 1;
	else 
		return 0;

}
/** 
  * @brief  过台阶
  * @note
  * @param  None
  * @retval None
  */
uint8_t StepOver_RED(void)
{
	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;
	lf_rb.limit=350.0f;
	if(gaitCnt == 13)
	{
		if(stepCnt>=5)
			closeLoopForward_y=25.0f;
        closeLoopForward_x=0.0f;
		goBack_flag_y=0;
		
		if(stepCnt >= 2)
		{
			goBack_pos_x = 13.f;
			goBack_flag_x = 1;
		}
	}
	// 1 4 支撑	x轴支撑
	else if(gaitCnt == 14)
	{
		lf_rb.limit=300.0f;
        closeLoopForward_y=0.0f;
		if(stepCnt>=8)
			closeLoopForward_x=250.0f;
        if(stepCnt>=6)
        {
            goBack_pos_y = 50.f;
			goBack_flag_y = 1; 
            
        }
        
		openLoopVel1 = 0;
	}
  //2 3支撑
	else if(gaitCnt == 15) 
	{

		rf_lb.limit=300.0f;
		if(stepCnt >= 6)
		{
			goBack_pos_x = 45.f;//2调x轴,1调y轴
			goBack_flag_x = 1;

		}
		
		if(stepCnt >= 10)
		{
			closeLoopForward_y = -100;
		}
	
	}
	//1 4 支撑
	else if(gaitCnt == 16)
	{
		closeLoopForward_x = -180;
		lf_rb.limit=350.0f;	  
		if(stepCnt >= 6)
		{
			goBack_pos_y = 40.f;
			goBack_flag_y = 1;
		}
	}
	//2 3支撑 X轴支撑
	else if(gaitCnt == 17)
	{
		rf_lb.k=3000.0f;
		rf_lb.b=700.0f;
		rf_lb.limit=350.0f;
        if(STRCnt<=18)
			closeLoopForward_y=230.0f;
        else 
			closeLoopForward_y =-100.0f;
		if(stepCnt >= 6)
		{
			goBack_pos_x = 35.f;
			goBack_flag_x = 1;
		}		
	
	}
	
	switch(stepStep)
	{
		/*1号腿上台*///13
		case 0:
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;
			
			/*当前想到达的位置*/
			one_hip =stepup_hip_swing_trans[stepCnt];
			one_knee=stepup_knee_swing_trans[stepCnt];
		
			two_hip =step_hip_support_trans[stepCnt];
			two_knee=step_knee_support_trans[stepCnt];
		
			thr_hip =step_hip_support_trans[stepCnt];
			thr_knee=step_knee_support_trans[stepCnt];
		
			for_hip =step_hip_swing_trans[stepCnt];
			for_knee=step_knee_swing_trans[stepCnt];
			
			/*下一个想到达的位置*/			
			one_hip_next =stepup_hip_swing_trans[stepCnt+1];
		    one_knee_next=stepup_knee_swing_trans[stepCnt+1];
		            
		    two_hip_next =step_hip_support_trans[stepCnt+1];
		    two_knee_next=step_knee_support_trans[stepCnt+1];
		            
		    thr_hip_next =step_hip_support_trans[stepCnt+1];
		    thr_knee_next=step_knee_support_trans[stepCnt+1];
		            
		    for_hip_next =step_hip_swing_trans[stepCnt+1];
		    for_knee_next=step_knee_swing_trans[stepCnt+1];
					
			stepCnt++;
			if(stepCnt >= 25)
			{
				goBack_flag_x=0;
				closeLoopForward_y = 0;
				stepStep++;
				stepCnt=0;
				gaitCnt++;
			}
			break;
			
		/*2、3号腿上台*///14
		case 1:
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;
			
			/*当前想到达的位置*/
			one_hip =onStep_hip_support_1[stepCnt];
			one_knee=onStep_knee_support_1[stepCnt];
		
			two_hip =stepup_hip_swing_2[stepCnt];
			two_knee=stepup_knee_swing_2[stepCnt];
		
			thr_hip =stepup_hip_swing_2[stepCnt];
			thr_knee=stepup_knee_swing_2[stepCnt];
		
			for_hip =stright_hip_support_4[stepCnt];
			for_knee=stright_knee_support_4[stepCnt];
			
			/*下一个想到达的位置*/			
			one_hip_next =onStep_hip_support_1[stepCnt+1];
		    one_knee_next=onStep_knee_support_1[stepCnt+1];
		                 
		    two_hip_next =stepup_hip_swing_2[stepCnt+1];
		    two_knee_next=stepup_knee_swing_2[stepCnt+1];
		                 
		    thr_hip_next =stepup_hip_swing_2[stepCnt+1];
		    thr_knee_next=stepup_knee_swing_2[stepCnt+1];
		                 
		    for_hip_next =stright_hip_support_4[stepCnt+1];
		    for_knee_next=stright_knee_support_4[stepCnt+1];
		
			stepCnt++;
			if(stepCnt >= 25)
			{
				goBack_flag_y=0;
				closeLoopForward_x = 0;
				stepStep++;
				stepCnt=0;
				gaitCnt++;
			}
			break;
			
		/*1号腿下台，4号腿上台*///15
		case 2:
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;
			
			/*当前想到达的位置*/
			one_hip =stepdown_hip_swing_1[stepCnt];
			one_knee=stepdown_knee_swing_1[stepCnt];
		
			two_hip =onStep_hip_support_2[stepCnt];
			two_knee=onStep_knee_support_2[stepCnt];
		
			thr_hip =onStep_hip_support_2[stepCnt];
			thr_knee=onStep_knee_support_2[stepCnt];
		
			for_hip =stepup_hip_swing_4[stepCnt];
			for_knee=stepup_knee_swing_4[stepCnt];
			
			/*下一个想到达的位置*/
			one_hip_next =stepdown_hip_swing_1[stepCnt+1];
			one_knee_next=stepdown_knee_swing_1[stepCnt+1];
                         
			two_hip_next =onStep_hip_support_2[stepCnt+1];		
			two_knee_next=onStep_knee_support_2[stepCnt+1];		
			             		
			thr_hip_next =onStep_hip_support_2[stepCnt+1];		
			thr_knee_next=onStep_knee_support_2[stepCnt+1];		
			             		
			for_hip_next =stepup_hip_swing_4[stepCnt+1];		
			for_knee_next=stepup_knee_swing_4[stepCnt+1];		

			stepCnt++;
			if(stepCnt >= 25)
			{
				closeLoopForward_y = 0;
				goBack_flag_x = 0; 
				stepStep++;
				stepCnt=0;
				gaitCnt++;
			}
			break;
			
		/*2、3号腿下台*///16
		case 3:
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;
			
			/*当前想到达的位置*/
			one_hip =step_hip_support[stepCnt];
			one_knee=step_knee_support[stepCnt];
		
			two_hip =stepdown_hip_swing[stepCnt];
			two_knee=stepdown_knee_swing[stepCnt];
		
			thr_hip =stepdown_hip_swing[stepCnt];
			thr_knee=stepdown_knee_swing[stepCnt];
		
			for_hip =onStep_hip_support[stepCnt];
			for_knee=onStep_knee_support[stepCnt];
			
			/*下一个想到达的位置*/
			one_hip_next =step_hip_support[stepCnt+1];
            one_knee_next=step_knee_support[stepCnt+1];
                         
            two_hip_next =stepdown_hip_swing[stepCnt+1];
            two_knee_next=stepdown_knee_swing[stepCnt+1];
                         
            thr_hip_next =stepdown_hip_swing[stepCnt+1];
            thr_knee_next=stepdown_knee_swing[stepCnt+1];
                         
            for_hip_next =onStep_hip_support[stepCnt+1];
            for_knee_next=onStep_knee_support[stepCnt+1];
			
			stepCnt++;
			if(stepCnt >= 25)
			{
				goBack_flag_y=0;
				closeLoopForward_x = 0;
				stepStep++;
				stepCnt=0;
				gaitCnt++;
			}
			break;
			
		/*4号腿下台*/
		case 4://17
			
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;
	
			/*当前想到达的位置*/
			one_hip =step_hip_swing[stepCnt];
			one_knee=step_knee_swing[stepCnt];
		
			two_hip =step_hip_support[stepCnt];
			two_knee=step_knee_support[stepCnt];
		
			thr_hip =step_hip_support[stepCnt];
			thr_knee=step_knee_support[stepCnt];
		
			for_hip =stepdown_hip_swing[stepCnt];
			for_knee=stepdown_knee_swing[stepCnt];
			
			/*下一个想到达的位置*/			
			one_hip_next =step_hip_swing[stepCnt+1];
		    one_knee_next=step_knee_swing[stepCnt+1];
		                 
		    two_hip_next =step_hip_support[stepCnt+1];
		    two_knee_next=step_knee_support[stepCnt+1];
		                 
		    thr_hip_next =step_hip_support[stepCnt+1];
		    thr_knee_next=step_knee_support[stepCnt+1];
		                 
		    for_hip_next =stepdown_hip_swing[stepCnt+1];
		    for_knee_next=stepdown_knee_swing[stepCnt+1];
		
			stepCnt++;
			if(stepCnt >= 25)
			{
				goBack_flag_x = 0;
				closeLoopForward_y = 0;
				stepStep=0;
				stepCnt=0;
				gaitCnt++;
			}
			break;
		default:
			break;
	}
	
	TurnMotorControl();
	
	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);
	
	/*1号腿上台阶时航向向左转45度*/
	if(gaitCnt == 13)
		Oblique(45.0f*PI/180.0f);
	if(gaitCnt == 17)
		Oblique(STRANGLE_RED*PI/180.0f);
	
	if(gaitCnt >= 18)//180
		return 1;
	else
		return 0;	
}

/** 
  * @brief  台阶到过绳的过渡
  * @note
  * @param  None
  * @retval None
  */

uint8_t STRTrans_RED(void)
{
    extern uint8_t STRacc;
	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;
	quadrupedControl.legAngle=STRANGLE_RED*PI/180.0f;
	/*距离调整*/
	
	rf_lb.k=3000.0f;
	rf_lb.b=700.0f;
	rf_lb.limit=350.0f;	
	
	lf_rb.k=3000.0f;
	lf_rb.b=700.0f;
    rf_lb.limit=350.0f;	
    
	if(gaitCnt == 18)
	{
		if(STRCnt > 5)
			closeLoopForward_x = -300.f;
		closeLoopForward_y = 0.f;
	
		if(STRCnt >= 5)
		{
			goBack_pos_y = 55.f;
			goBack_flag_y = 1;
		}
	}	
	else if(gaitCnt == 19)
	{
        
		closeLoopForward_x = 0.f;
        if(STRacc==2)        
        {
            if(STRCnt<=11)
                closeLoopForward_y = 150.f;
            else 
                closeLoopForward_y=-370.f;
        }
        else if(STRacc==1)
        {
            if(STRCnt>=10)
            closeLoopForward_y=-400.0f;
        }            
		if(STRCnt >= 6)
		{
			goBack_pos_x = 35.f;
			goBack_flag_x = 1;
		}		
	}
    
    //14zhi成
	else if(gaitCnt == 20)
	{
		if(STRCnt > 6)
			closeLoopForward_x = -170.f;
		closeLoopForward_y = 0.f;
		if(STRCnt >= 6)
		{
            if(STRacc==2)
                goBack_pos_y = 15.f;
            if(STRacc==1)
                goBack_pos_y = 68.f;
			goBack_flag_y = 1;
		}		
	}
    

	else if(gaitCnt == 21)
	{
		closeLoopForward_x = 0.f;
        if(STRacc==2)
        {
            if(STRCnt>=4)
            closeLoopForward_y = 200.f;
            else closeLoopForward_y = 0.f;
        }
        if(STRacc==1)
        {
            if(STRCnt>=8)
            closeLoopForward_y = -200.f;
            else closeLoopForward_y = 0.f;            
        }
        if(STRCnt >= 2)
		{
			goBack_pos_x = 60.f;
			goBack_flag_x = 1;
		}		
	}	
	switch(STRStep)
	{
		case 0://18
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;
		
			/*当前想到达的位置*/
			one_hip	=STR_hip_support_trans_1[STRCnt];
			one_knee=STR_knee_support_trans_1[STRCnt];
		    		
			two_hip	=STR_hip_swing_trans_1[STRCnt];
			two_knee=STR_knee_swing_trans_1[STRCnt];
					
			thr_hip	=STR_hip_swing_trans_1[STRCnt];
			thr_knee=STR_knee_swing_trans_1[STRCnt];
					
			for_hip	=STR_hip_support_trans_1[STRCnt];
			for_knee=STR_knee_support_trans_1[STRCnt];
			
			/*下一个想到达的位置*/
			one_hip_next =STR_hip_support_trans_1[STRCnt+1];
		    one_knee_next=STR_knee_support_trans_1[STRCnt+1];
			              
		    two_hip_next =STR_hip_swing_trans_1[STRCnt+1];
		    two_knee_next=STR_knee_swing_trans_1[STRCnt+1];
		                  
		    thr_hip_next =STR_hip_swing_trans_1[STRCnt+1];
		    thr_knee_next=STR_knee_swing_trans_1[STRCnt+1];
		                  
		    for_hip_next =STR_hip_support_trans_1[STRCnt+1];
		    for_knee_next=STR_knee_support_trans_1[STRCnt+1];
		
			STRCnt++;
			if(STRCnt >= 25)
			{
				closeLoopForward_x=0;
				goBack_flag_y = 0;
				STRStep++;
				STRCnt=0;
				gaitCnt++;
			}
			break;
		case 1:
			legOne.standFlag=legFor.standFlag=0;;
			legTwo.standFlag=legThr.standFlag=1;
			
			/*当前想到达的位置*/
			one_hip	=STR_hip_swing[STRCnt];
			one_knee=STR_knee_swing[STRCnt];
		    		
			two_hip	=STR_hip_support[STRCnt];
			two_knee=STR_knee_support[STRCnt];
		    		
			thr_hip	=STR_hip_support[STRCnt];
			thr_knee=STR_knee_support[STRCnt];
		    		
			for_hip	=STR_hip_swing[STRCnt];
			for_knee=STR_knee_swing[STRCnt];
			
			/*下一个想到达的位置*/
			one_hip_next =STR_hip_swing[STRCnt+1];
			one_knee_next=STR_knee_swing[STRCnt+1];
		                  
		    two_hip_next =STR_hip_support[STRCnt+1];
		    two_knee_next=STR_knee_support[STRCnt+1];
		                  
		    thr_hip_next =STR_hip_support[STRCnt+1];
		    thr_knee_next=STR_knee_support[STRCnt+1];
		                  
		    for_hip_next =STR_hip_swing[STRCnt+1];
		    for_knee_next=STR_knee_swing[STRCnt+1];
		
			STRCnt++;
			if(STRCnt >= 25)
			{
				closeLoopForward_y=0;
				goBack_flag_x = 0;
				STRStep++;
				STRCnt=0;
				gaitCnt++;
			}
			break;
		case 2://18
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;
			
			/*当前想到达的位置*/
			one_hip	=STR_hip_support[STRCnt];
			one_knee=STR_knee_support[STRCnt];
		    		
			two_hip	=STR_hip_swing[STRCnt];
			two_knee=STR_knee_swing[STRCnt];
					
			thr_hip	=STR_hip_swing[STRCnt];
			thr_knee=STR_knee_swing[STRCnt];
					
			for_hip	=STR_hip_support[STRCnt];
			for_knee=STR_knee_support[STRCnt];
		
			/*下一个想到达的位置*/
			one_hip_next =STR_hip_support[STRCnt+1];
		    one_knee_next=STR_knee_support[STRCnt+1];
		                  
		    two_hip_next =STR_hip_swing[STRCnt+1];
		    two_knee_next=STR_knee_swing[STRCnt+1];
		                  
		    thr_hip_next =STR_hip_swing[STRCnt+1];
		    thr_knee_next=STR_knee_swing[STRCnt+1];
		                  
		    for_hip_next =STR_hip_support[STRCnt+1];
		    for_knee_next=STR_knee_support[STRCnt+1];

			STRCnt++;
			if(STRCnt >= 25)
			{
				closeLoopForward_x=0;
				goBack_flag_y = 0;
				STRStep++;
				STRCnt=0;
				gaitCnt++;
			}
			break;
		case 3:
			legOne.standFlag=legFor.standFlag=0;;
			legTwo.standFlag=legThr.standFlag=1;
		
			/*当前想到达的位置*/
			one_hip	=STR_hip_swing_trans_2[STRCnt];
			one_knee=STR_knee_swing_trans_2[STRCnt];
		    		
			two_hip	=STR_hip_support_trans_2[STRCnt];
			two_knee=STR_knee_support_trans_2[STRCnt];
		    		
			thr_hip	=STR_hip_support_trans_2[STRCnt];
			thr_knee=STR_knee_support_trans_2[STRCnt];
		    		
			for_hip	=STR_hip_swing_trans_2[STRCnt];
			for_knee=STR_knee_swing_trans_2[STRCnt];
		
			/*下一个想到达的位置*/	
			one_hip_next =STR_hip_swing_trans_2[STRCnt+1];
		    one_knee_next=STR_knee_swing_trans_2[STRCnt+1];
		                  
		    two_hip_next =STR_hip_support_trans_2[STRCnt+1];
		    two_knee_next=STR_knee_support_trans_2[STRCnt+1];
		                  
		    thr_hip_next =STR_hip_support_trans_2[STRCnt+1];
		    thr_knee_next=STR_knee_support_trans_2[STRCnt+1];
		                  
		    for_hip_next =STR_hip_swing_trans_2[STRCnt+1];
		    for_knee_next=STR_knee_swing_trans_2[STRCnt+1];
		
		
			STRCnt++;
			if(STRCnt >= 25)
			{
				closeLoopForward_y=0;
				goBack_flag_x = 0;
				STRStep=0;
				STRCnt=0;
				gaitCnt++;
			}
			break;
		default:
			break;
			
	}
	
	TurnMotorControl();
	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);
	
	
	if(gaitCnt >= 22)//22
		return 1;
	else 
		return 0;
}	
/** 
  * @brief  跨绳子
  * @note
  * @param  None
  * @retval None
  */

uint8_t CrossRope_RED(void)
{

	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;
	if(gaitCnt == 22)
	{
		if(ropeCnt >= 5)
			closeLoopForward_x = -550.f;
		closeLoopForward_y = 0.f;
		
		lf_rb.k=3200.0f;//调X轴
		lf_rb.b=800.0f;
		lf_rb.limit=450.0f;	
		
		if(ropeCnt >= 6)
		{
            
			goBack_pos_y = 55.f;
			goBack_flag_y = 1;
		}
	}	
	else if(gaitCnt == 23)
	{

		closeLoopForward_x = 0.f;
		if(ropeCnt >= 6)
			closeLoopForward_y = -20.f;
		
		rf_lb.k=3200.0f;//调Y轴
		rf_lb.b=800.0f;
		rf_lb.limit=450.0f;	
		
		if(ropeCnt >= 6)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 24)
	{
		closeLoopForward_x = 170.0f;
		closeLoopForward_y = 0.f;
		
		if(ropeCnt >= 7)
		{
			goBack_pos_y = 55.f;
			goBack_flag_y = 1;
		}
	}	
	else if(gaitCnt == 25)
	{
		closeLoopForward_x = 0.f;
		if(ropeCnt >= 6)
			closeLoopForward_y = 40.f;
		
		if(ropeCnt >= 6)
		{
			goBack_pos_x = 68.f;
			goBack_flag_x = 1;
		}
	}
	
	else if(gaitCnt == 26)
	{
		closeLoopForward_x = 0.f;
		closeLoopForward_y = 0.f;
		
		if(ropeCnt >= 6)
		{
			goBack_pos_y = 55.f;
			goBack_flag_y = 1;
		}
	}	
	else if(gaitCnt == 27)
	{

		closeLoopForward_x = 0.f;
		closeLoopForward_y = -150.f;
		
		if(ropeCnt >= 6)
		{
			goBack_pos_x = 68.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 28)
	{
		closeLoopForward_x = 150.0f;
		closeLoopForward_y = 0.f;
		
		if(ropeCnt >= 7)
		{
			goBack_pos_y = 55.f;
			goBack_flag_y = 1;
		}
	}	
	else if(gaitCnt == 29)
	{
		closeLoopForward_x = 0.f;
		closeLoopForward_y = 20.f;
		
		if(ropeCnt >= 6)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}		
	
	switch(ropeStep)
	{
		/*2号腿过绳过渡*/
		case 0://22
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			/*当前想到达的位置*/
			one_hip	=rope_hip_trans_support[ropeCnt];
			one_knee=rope_knee_trans_support[ropeCnt];
		
			two_hip	=rope_hip_trans_LH[ropeCnt];
			two_knee=rope_knee_trans_LH[ropeCnt];
		
			thr_hip	=rope_hip_swing[ropeCnt];
			thr_knee=rope_knee_swing[ropeCnt];
		
			for_hip	=rope_hip_trans_support[ropeCnt];
			for_knee=rope_knee_trans_support[ropeCnt];

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_trans_support[ropeCnt+1];
		    one_knee_next=rope_knee_trans_support[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_trans_LH[ropeCnt+1];
		    two_knee_next=rope_knee_trans_LH[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_swing[ropeCnt+1];
		    thr_knee_next=rope_knee_swing[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_trans_support[ropeCnt+1];
		    for_knee_next=rope_knee_trans_support[ropeCnt+1];
			
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_y = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
			
		/*1号腿过绳*/
		case 1://23
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_HL_swing[ropeCnt];
			one_knee=rope_knee_HL_swing[ropeCnt];
		    
			two_hip	=rope_hip_support[ropeCnt];
			two_knee=rope_knee_support[ropeCnt];
		    
			thr_hip	=rope_hip_support[ropeCnt];
			thr_knee=rope_knee_support[ropeCnt];
		    
			for_hip	=rope_hip_swing[ropeCnt];
			for_knee=rope_knee_swing[ropeCnt];

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_HL_swing[ropeCnt+1];
		    one_knee_next=rope_knee_HL_swing[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_support[ropeCnt+1];
		    two_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_support[ropeCnt+1];
		    thr_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_swing[ropeCnt+1];
		    for_knee_next=rope_knee_swing[ropeCnt+1];
			
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_x = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
			
		/*3号腿过绳*/
		case 2://24
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_support[ropeCnt];
			one_knee=rope_knee_support[ropeCnt];
		    
			two_hip	=rope_hip_swing[ropeCnt];
			two_knee=rope_knee_swing[ropeCnt];
		    
			thr_hip	=rope_hip_LH_swing[ropeCnt];
			thr_knee=rope_knee_LH_swing[ropeCnt];
		    
			for_hip	=rope_hip_support[ropeCnt];
			for_knee=rope_knee_support[ropeCnt];

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_support[ropeCnt+1];
		    one_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_swing[ropeCnt+1];
		    two_knee_next=rope_knee_swing[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_LH_swing[ropeCnt+1];
		    thr_knee_next=rope_knee_LH_swing[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_support[ropeCnt+1];
		    for_knee_next=rope_knee_support[ropeCnt+1];
					
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_y = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
			
		/*4号腿过绳*/
		case 3://25
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_swing[ropeCnt];
			one_knee=rope_knee_swing[ropeCnt];
		    
			two_hip	=rope_hip_support[ropeCnt];
			two_knee=rope_knee_support[ropeCnt];
		    
			thr_hip	=rope_hip_support[ropeCnt];
			thr_knee=rope_knee_support[ropeCnt];
		    
			for_hip	=rope_hip_HL_swing[ropeCnt];	
			for_knee=rope_knee_HL_swing[ropeCnt];	
		
			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_swing[ropeCnt+1];
		    one_knee_next=rope_knee_swing[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_support[ropeCnt+1];
		    two_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_support[ropeCnt+1];
		    thr_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_HL_swing[ropeCnt+1];	
		    for_knee_next=rope_knee_HL_swing[ropeCnt+1];	
			
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_x = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
		/*2号腿过绳*/
		case 4://26
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_support[ropeCnt];
			one_knee=rope_knee_support[ropeCnt];
		    
			two_hip	=rope_hip_LH_swing[ropeCnt];
			two_knee=rope_knee_LH_swing[ropeCnt];
		    
			thr_hip	=rope_hip_swing[ropeCnt];
			thr_knee=rope_knee_swing[ropeCnt];
		    
			for_hip	=rope_hip_support[ropeCnt];
			for_knee=rope_knee_support[ropeCnt];

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_support[ropeCnt+1];
		    one_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_LH_swing[ropeCnt+1];
		    two_knee_next=rope_knee_LH_swing[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_swing[ropeCnt+1];
		    thr_knee_next=rope_knee_swing[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_support[ropeCnt+1];
		    for_knee_next=rope_knee_support[ropeCnt+1];
					
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_y = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
		/*1号腿过绳*/
		case 5://27
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_HL_swing[ropeCnt];
			one_knee=rope_knee_HL_swing[ropeCnt];
		    
			two_hip	=rope_hip_support[ropeCnt];
			two_knee=rope_knee_support[ropeCnt];
		    
			thr_hip	=rope_hip_support[ropeCnt];
			thr_knee=rope_knee_support[ropeCnt];
		    
			for_hip	=rope_hip_swing[ropeCnt];
			for_knee=rope_knee_swing[ropeCnt];

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_HL_swing[ropeCnt+1];
		    one_knee_next=rope_knee_HL_swing[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_support[ropeCnt+1];
		    two_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_support[ropeCnt+1];
		    thr_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_swing[ropeCnt+1];
		    for_knee_next=rope_knee_swing[ropeCnt+1];
					
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_x = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
			
		/*3号腿过绳*/
		case 6://28
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_support[ropeCnt];
			one_knee=rope_knee_support[ropeCnt];
		    
			two_hip	=rope_hip_swing[ropeCnt];
			two_knee=rope_knee_swing[ropeCnt];
		    
			thr_hip	=rope_hip_LH_swing[ropeCnt];
			thr_knee=rope_knee_LH_swing[ropeCnt];
		    
			for_hip	=rope_hip_support[ropeCnt];
			for_knee=rope_knee_support[ropeCnt];

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_support[ropeCnt+1];
		    one_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_swing[ropeCnt+1];
		    two_knee_next=rope_knee_swing[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_LH_swing[ropeCnt+1];
		    thr_knee_next=rope_knee_LH_swing[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_support[ropeCnt+1];
		    for_knee_next=rope_knee_support[ropeCnt+1];
			
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_y = 0;
				ropeStep++;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
			
		/*4号腿过绳*/
		case 7://29
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			/*当前想到达的位置*/		
			one_hip	=rope_hip_swing[ropeCnt];
			one_knee=rope_knee_swing[ropeCnt];
		    
			two_hip	=rope_hip_support[ropeCnt];
			two_knee=rope_knee_support[ropeCnt];
		    
			thr_hip	=rope_hip_support[ropeCnt];
			thr_knee=rope_knee_support[ropeCnt];
		    
			for_hip	=rope_hip_HL_swing[ropeCnt];	
			for_knee=rope_knee_HL_swing[ropeCnt];	

			/*下一个想到达的位置*/	
			one_hip_next =rope_hip_swing[ropeCnt+1];
		    one_knee_next=rope_knee_swing[ropeCnt+1];
		                  
		    two_hip_next =rope_hip_support[ropeCnt+1];
		    two_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    thr_hip_next =rope_hip_support[ropeCnt+1];
		    thr_knee_next=rope_knee_support[ropeCnt+1];
		                  
		    for_hip_next =rope_hip_HL_swing[ropeCnt+1];	
		    for_knee_next=rope_knee_HL_swing[ropeCnt+1];	
					
			ropeCnt++;
			if(ropeCnt >= 25)
			{
				goBack_flag_x = 0;
				ropeStep=0;
				ropeCnt=0;
				gaitCnt++;
			}
			break;
		default:
			break;
	}
	
	TurnMotorControl();
	
	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);

	/*1、2号腿过第一条绳时角度为向左8度*/
	if(gaitCnt == 22)
		Oblique(-ROPEANGLE_RED_1*PI/180.0f);
	if(gaitCnt == 26)
		Oblique(-ROPEANGLE_RED_2*PI/180.0f);
    
   /*当四号腿过完绳2之后反1*/
	if(gaitCnt >= 30)
		return 1;
	else 
		return 0;
}
/** 
  * @brief  驿站换向
  * @note
  * @param  None
  * @retval None
  */
extern uint8_t closeLoopFlag;
uint8_t Relay_RED(void)
{

	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;
	
	closeLoopForward_x = 0.0f;
	closeLoopForward_y = 0.0f;
	
	if(relayCnt >= 6)
	{
		goBack_pos_y = 40.f;
		goBack_flag_y = 1;
	}
			
	legOne.standFlag=legFor.standFlag=1;
	legTwo.standFlag=legThr.standFlag=0;

	one_hip	=slope_hip_trans_support[relayCnt]; 
	one_knee=slope_knee_trans_support[relayCnt];
	
	two_hip	=slope_hip_trans_swing[relayCnt]; 
	two_knee=slope_knee_trans_swing[relayCnt];
	
	thr_hip	=slope_hip_trans_swing[relayCnt]; 
	thr_knee=slope_knee_trans_swing[relayCnt];
	
	for_hip	=slope_hip_trans_support[relayCnt]; 
	for_knee=slope_knee_trans_support[relayCnt];
	
	/*下一个想到达的位置*/	
	one_hip_next =slope_hip_trans_support[relayCnt+1]; 
	one_knee_next=slope_knee_trans_support[relayCnt+1];
				  
	two_hip_next =slope_hip_trans_swing[relayCnt+1]; 
	two_knee_next=slope_knee_trans_swing[relayCnt+1];
				  
	thr_hip_next =slope_hip_trans_swing[relayCnt+1]; 
	thr_knee_next=slope_knee_trans_swing[relayCnt+1];
				  
	for_hip_next =slope_hip_trans_support[relayCnt+1]; 
	for_knee_next=slope_knee_trans_support[relayCnt+1];
			
	relayCnt++;
	if(relayCnt >= 25)
	{
		closeLoopFlag=0;
		
        quadrupedControl.yawAdjust=0;
		goBack_flag_y = 0;
		relayCnt--;
        SendStopReady();
		if(visual.goFlag == 1 || Manualtrigger() ==1)
		{
			closeLoopFlag=1;
			gaitCnt++;
			relayCnt=0;
		}
	}
	if(relayCnt >= 5)
		quadrupedControl.legAngle=-90.0f*PI/180.0f;
	
	
	
	TurnMotorControl();

	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);
	
	if(gaitCnt >= 31)
		return 1;
	else
		return 0;
}


/** 
  * @brief	上坡
  * @note
  * @param  None
  * @retval None
  */

uint8_t Uphill_RED(void)
{

	float one_hip=0,two_hip=0,thr_hip=0,for_hip=0,one_knee=0,two_knee=0,thr_knee=0,for_knee=0,\
		  one_hip_next=0,two_hip_next=0,thr_hip_next=0,for_hip_next=0,one_knee_next=0,two_knee_next=0,thr_knee_next=0,for_knee_next=0;
	quadrupedControl.legAngle=-90.0f*PI/180.0f;
	
	openLoopVel1=0;
	openLoopVel2=0;
	lf_rb.k=3100.0f;
	lf_rb.b=730.0f;
	lf_rb.limit=400.0f;
	
	rf_lb.k=3200.0f;
	rf_lb.b=770.0f;
	rf_lb.limit=420.0f;
	
	if(gaitCnt == 31)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -190.0f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 32)
	{
        
		closeLoopForward_x = -140.0f;
		closeLoopForward_y = 0.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_y = 68.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 33)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -140.0f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 50.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 34)
	{
		closeLoopForward_x = -100.0f;
		closeLoopForward_y = 0.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_y = 40.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 35)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -120.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 60.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 36)
	{
		closeLoopForward_x = -130.0f;
		closeLoopForward_y = 0.0f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_y = 45.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 37)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -70.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 55.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 38)
	{
		closeLoopForward_x = -55.0f;
		closeLoopForward_y = 0.0f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_y = 45.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 39)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -150.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 15.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 40)
	{
		closeLoopForward_x = 280.0f;
		closeLoopForward_y = 0.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_y = 50.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 41)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -30.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 50.f;
			goBack_flag_x = 1;
		}
	}
	else if(gaitCnt == 42)
	{
		closeLoopForward_x = 20.0f;
		closeLoopForward_y = 0.0f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_y = 68.f;
			goBack_flag_y = 1;
		}
	}
	else if(gaitCnt == 43)
	{
		closeLoopForward_x = 0.0f;
		closeLoopForward_y = -100.f;
		
		if(hillCnt >= 5)
		{
			goBack_pos_x = 40.f;
			goBack_flag_x = 1;
		}
	}
	
	switch(hillStep)
	{
		case 0:
			desireAngle_X=angleTransToX((-0.1261f*hillCnt-0.3782f)*PI/180.0f);
			desireAngle_Y=angleTransToY((-0.1261f*hillCnt-0.3782f)*PI/180.0f);
		
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			one_hip	=PTS_one_hip_1[hillCnt];
			one_knee=PTS_one_knee_1[hillCnt];
		    
			two_hip	=PTS_two_hip_1[hillCnt];
			two_knee=PTS_two_knee_1[hillCnt];
		    
			thr_hip	=PTS_thr_hip_1[hillCnt];	
			thr_knee=PTS_thr_knee_1[hillCnt];	
		    
			for_hip	=PTS_for_hip_1[hillCnt];
			for_knee=PTS_for_knee_1[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =PTS_one_hip_1[hillCnt+1];
		    one_knee_next=PTS_one_knee_1[hillCnt+1];
		                  
		    two_hip_next =PTS_two_hip_1[hillCnt+1];
		    two_knee_next=PTS_two_knee_1[hillCnt+1];
		                  
		    thr_hip_next =PTS_thr_hip_1[hillCnt+1];	
		    thr_knee_next=PTS_thr_knee_1[hillCnt+1];
		                  
		    for_hip_next =PTS_for_hip_1[hillCnt+1];
		    for_knee_next=PTS_for_knee_1[hillCnt+1];
			
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_x = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 1:
			desireAngle_X=angleTransToX((-0.2794f*hillCnt-3.0586f)*PI/180.0f);
			desireAngle_Y=angleTransToY((-0.2794f*hillCnt-3.0586f)*PI/180.0f);
		
			
			
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			one_hip	=PTS_one_hip_2[hillCnt];
			one_knee=PTS_one_knee_2[hillCnt];
		                 
			two_hip	=PTS_two_hip_2[hillCnt];
			two_knee=PTS_two_knee_2[hillCnt];
		                 
			thr_hip	=PTS_thr_hip_2[hillCnt];	
			thr_knee=PTS_thr_knee_2[hillCnt];	
		                 
			for_hip	=PTS_for_hip_2[hillCnt];
			for_knee=PTS_for_knee_2[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =PTS_one_hip_2[hillCnt+1];
		    one_knee_next=PTS_one_knee_2[hillCnt+1];
		                      
		    two_hip_next =PTS_two_hip_2[hillCnt+1];
		    two_knee_next=PTS_two_knee_2[hillCnt+1];
		                      
		    thr_hip_next =PTS_thr_hip_2[hillCnt+1];	
		    thr_knee_next=PTS_thr_knee_2[hillCnt+1];	
		                      
		    for_hip_next =PTS_for_hip_2[hillCnt+1];
		    for_knee_next=PTS_for_knee_2[hillCnt+1];
						
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_y = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 2:
			desireAngle_X=angleTransToX((-0.2835f*hillCnt-8.6483f)*PI/180.0f);
			desireAngle_Y=angleTransToY((-0.2835f*hillCnt-8.6483f)*PI/180.0f);

			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			one_hip	=PTS_one_hip_3[hillCnt];
			one_knee=PTS_one_knee_3[hillCnt];
		                 
			two_hip	=PTS_two_hip_3[hillCnt];
			two_knee=PTS_two_knee_3[hillCnt];
		                 
			thr_hip	=PTS_thr_hip_3[hillCnt];	
			thr_knee=PTS_thr_knee_3[hillCnt];	
		                 
			for_hip	=PTS_for_hip_3[hillCnt];
			for_knee=PTS_for_knee_3[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =PTS_one_hip_3[hillCnt+1];
		    one_knee_next=PTS_one_knee_3[hillCnt+1];
		                      
		    two_hip_next =PTS_two_hip_3[hillCnt+1];
		    two_knee_next=PTS_two_knee_3[hillCnt+1];
		                      
		    thr_hip_next =PTS_thr_hip_3[hillCnt+1];	
		    thr_knee_next=PTS_thr_knee_3[hillCnt+1];	
		                      
		    for_hip_next =PTS_for_hip_3[hillCnt+1];
		    for_knee_next=PTS_for_knee_3[hillCnt+1];
						
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_x = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 3:
			desireAngle_X=angleTransToX(-14.036f*PI/180.0f);
		    desireAngle_Y=angleTransToY(-14.036f*PI/180.0f);
		
			

			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			one_hip	=PTS_one_hip_4[hillCnt];
			one_knee=PTS_one_knee_4[hillCnt];
		                 
			two_hip	=PTS_two_hip_4[hillCnt];
			two_knee=PTS_two_knee_4[hillCnt];
		                 
			thr_hip	=PTS_thr_hip_4[hillCnt];	
			thr_knee=PTS_thr_knee_4[hillCnt];	
		                 
			for_hip	=PTS_for_hip_4[hillCnt];
			for_knee=PTS_for_knee_4[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =PTS_one_hip_4[hillCnt+1];
		    one_knee_next=PTS_one_knee_4[hillCnt+1];
		                      
		    two_hip_next =PTS_two_hip_4[hillCnt+1];
		    two_knee_next=PTS_two_knee_4[hillCnt+1];
		                      
		    thr_hip_next =PTS_thr_hip_4[hillCnt+1];	
		    thr_knee_next=PTS_thr_knee_4[hillCnt+1];
		                      
		    for_hip_next =PTS_for_hip_4[hillCnt+1];
		    for_knee_next=PTS_for_knee_4[hillCnt+1];
						
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_y = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 4:
			desireAngle_X=angleTransToX(-14.036f*PI/180.0f);
		    desireAngle_Y=angleTransToY(-14.036f*PI/180.0f);
		
			
			
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;
			
			one_hip	=slope_hip_swing[hillCnt];
			one_knee=slope_knee_swing[hillCnt];
		    
			two_hip	=slope_hip_support[hillCnt];
			two_knee=slope_knee_support[hillCnt];
		    
			thr_hip	=slope_hip_support[hillCnt];	
			thr_knee=slope_knee_support[hillCnt];	
		    
			for_hip	=slope_hip_swing[hillCnt];
			for_knee=slope_knee_swing[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =slope_hip_swing[hillCnt+1];
		    one_knee_next=slope_knee_swing[hillCnt+1];
		                  
		    two_hip_next =slope_hip_support[hillCnt+1];
		    two_knee_next=slope_knee_support[hillCnt+1];
		                  
		    thr_hip_next =slope_hip_support[hillCnt+1];	
		    thr_knee_next=slope_knee_support[hillCnt+1];	
		                  
		    for_hip_next =slope_hip_swing[hillCnt+1];
		    for_knee_next=slope_knee_swing[hillCnt+1];
						
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_x = 0;
				hillCnt=0;
				gaitCnt++;
				if(gaitCnt == 40)
					hillStep=6;
				else
					hillStep=5;
			}
			break;
		case 5:
			desireAngle_X=angleTransToX(-14.036f*PI/180.0f);
			desireAngle_Y=angleTransToY(-14.036f*PI/180.0f);
		
			
			
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			one_hip	=slope_hip_support[hillCnt];
			one_knee=slope_knee_support[hillCnt];
		             
			two_hip	=slope_hip_swing[hillCnt];
			two_knee=slope_knee_swing[hillCnt];
		             
			thr_hip	=slope_hip_swing[hillCnt];		
			thr_knee=slope_knee_swing[hillCnt];	
		             
			for_hip	=slope_hip_support[hillCnt];
			for_knee=slope_knee_support[hillCnt];
		
			/*下一个想到达的位置*/	
			one_hip_next =slope_hip_support[hillCnt+1];
		    one_knee_next=slope_knee_support[hillCnt+1];
		                  
		    two_hip_next =slope_hip_swing[hillCnt+1];
		    two_knee_next=slope_knee_swing[hillCnt+1];
		                  
		    thr_hip_next =slope_hip_swing[hillCnt+1];		
		    thr_knee_next=slope_knee_swing[hillCnt+1];	
		                  
		    for_hip_next =slope_hip_support[hillCnt+1];
		    for_knee_next=slope_knee_support[hillCnt+1];
						
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_y = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep=4;
			}
			break;
		
		case 6:
			desireAngle_X=angleTransToX(-14.036f*PI/180.0f);
		    desireAngle_Y=angleTransToY(-14.036f*PI/180.0f);
		
			
			
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			one_hip	=STP_one_hip_1[hillCnt];
			one_knee=STP_one_knee_1[hillCnt];
		                 
			two_hip	=STP_two_hip_1[hillCnt];
			two_knee=STP_two_knee_1[hillCnt];
		                 
			thr_hip	=STP_thr_hip_1[hillCnt];	
			thr_knee=STP_thr_knee_1[hillCnt];	
		                 
			for_hip	=STP_for_hip_1[hillCnt];
			for_knee=STP_for_knee_1[hillCnt];

			/*下一个想到达的位置*/	
			one_hip_next =STP_one_hip_1[hillCnt+1];
		    one_knee_next=STP_one_knee_1[hillCnt+1];
		                      
		    two_hip_next =STP_two_hip_1[hillCnt+1];
		    two_knee_next=STP_two_knee_1[hillCnt+1];
		                      
		    thr_hip_next =STP_thr_hip_1[hillCnt+1];	
		    thr_knee_next=STP_thr_knee_1[hillCnt+1];	
		                      
		    for_hip_next =STP_for_hip_1[hillCnt+1];
		    for_knee_next=STP_for_knee_1[hillCnt+1];
			
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_y = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 7:
			desireAngle_X=angleTransToX((0.2783f*hillCnt-14.036f)*PI/180.0f);
			desireAngle_Y=angleTransToY((0.2783f*hillCnt-14.036f)*PI/180.0f);
		
			
			legOne.standFlag=legFor.standFlag=0;
			legTwo.standFlag=legThr.standFlag=1;

			one_hip	=STP_one_hip_2[hillCnt];
			one_knee=STP_one_knee_2[hillCnt];
		                 
			two_hip	=STP_two_hip_2[hillCnt];
			two_knee=STP_two_knee_2[hillCnt];
		                 
			thr_hip	=STP_thr_hip_2[hillCnt];	
			thr_knee=STP_thr_knee_2[hillCnt];	
		                 
			for_hip	=STP_for_hip_2[hillCnt];
			for_knee=STP_for_knee_2[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =STP_one_hip_2[hillCnt+1];
		    one_knee_next=STP_one_knee_2[hillCnt+1];
		                      
		    two_hip_next =STP_two_hip_2[hillCnt+1];
		    two_knee_next=STP_two_knee_2[hillCnt+1];
		                      
		    thr_hip_next =STP_thr_hip_2[hillCnt+1];
		    thr_knee_next=STP_thr_knee_2[hillCnt+1];
		                      
		    for_hip_next =STP_for_hip_2[hillCnt+1];
		    for_knee_next=STP_for_knee_2[hillCnt+1];
			
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_x = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 8:
			desireAngle_X=angleTransToX((0.2809f*hillCnt-8.4688f)*PI/180.0f);
			desireAngle_Y=angleTransToY((0.2809f*hillCnt-8.4688f)*PI/180.0f);
		
			
			
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			one_hip	=STP_one_hip_3[hillCnt];
			one_knee=STP_one_knee_3[hillCnt];
		                 
			two_hip	=STP_two_hip_3[hillCnt];
			two_knee=STP_two_knee_3[hillCnt];
		                 
			thr_hip	=STP_thr_hip_3[hillCnt];	
			thr_knee=STP_thr_knee_3[hillCnt];	
		                 
			for_hip	=STP_for_hip_3[hillCnt];
			for_knee=STP_for_knee_3[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =STP_one_hip_3[hillCnt+1];
		    one_knee_next=STP_one_knee_3[hillCnt+1];
		                      
		    two_hip_next =STP_two_hip_3[hillCnt+1];
		    two_knee_next=STP_two_knee_3[hillCnt+1];
		                      
		    thr_hip_next =STP_thr_hip_3[hillCnt+1];	
		    thr_knee_next=STP_thr_knee_3[hillCnt+1];
		                      
		    for_hip_next =STP_for_hip_3[hillCnt+1];
		    for_knee_next=STP_for_knee_3[hillCnt+1];
			
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_y = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		case 9:
			if(hillCnt < 11)
			{
				desireAngle_X=angleTransToX((0.2848f*hillCnt-2.8482f)*PI/180.0f);
				desireAngle_Y=angleTransToY((0.2848f*hillCnt-2.8482f)*PI/180.0f);
			}
			else
			{
				desireAngle_X=0.0f;
				desireAngle_Y=0.0f;
			}
		
			
			legOne.standFlag=legFor.standFlag=1;
			legTwo.standFlag=legThr.standFlag=0;

			one_hip	=STP_one_hip_4[hillCnt];
			one_knee=STP_one_knee_4[hillCnt];
		                 
			two_hip	=STP_two_hip_4[hillCnt];
			two_knee=STP_two_knee_4[hillCnt];
		                 
			thr_hip	=STP_thr_hip_4[hillCnt];	
			thr_knee=STP_thr_knee_4[hillCnt];	
		                 
			for_hip	=STP_for_hip_4[hillCnt];
			for_knee=STP_for_knee_4[hillCnt];
			
			/*下一个想到达的位置*/	
			one_hip_next =STP_one_hip_4[hillCnt+1];
		    one_knee_next=STP_one_knee_4[hillCnt+1];
		                      
		    two_hip_next =STP_two_hip_4[hillCnt+1];
		    two_knee_next=STP_two_knee_4[hillCnt+1];
		                      
		    thr_hip_next =STP_thr_hip_4[hillCnt+1];	
		    thr_knee_next=STP_thr_knee_4[hillCnt+1];	
		                      
		    for_hip_next =STP_for_hip_4[hillCnt+1];
		    for_knee_next=STP_for_knee_4[hillCnt+1];
			
			hillCnt++;
			if(hillCnt >= 20)
			{
				goBack_flag_x = 0;
				hillCnt=0;
				gaitCnt++;
				hillStep++;
			}
			break;
		default:
			break;
	}
	
	TurnMotorControl();
	
	HipAndKneeMotorControl(one_hip,one_knee,two_hip,two_knee,thr_hip,thr_knee,for_hip,for_knee,\
	one_hip_next,one_knee_next,two_hip_next,two_knee_next,thr_hip_next,thr_knee_next,for_hip_next,for_knee_next);
	
    if(gaitCnt >= 44)
    {
        PosCrl2(CAN2, LIFT_UP_ID ,ABSOLUTE_MODE,-290000);
    }
	if(gaitCnt >= 44)
		return 1;
	else 
        return 0;
}
/** 
  * @brief	停止
  * @note
  * @param  None
  * @retval None 
  */
void Stop(void)
{
	VelCrl(CAN1,legOne.ID,0,0);
	VelCrl(CAN1,legTwo.ID,0,0);
	VelCrl(CAN1,legThr.ID,0,0);
	VelCrl(CAN1,legFor.ID,0,0);
}



 

/** 
  * @brief  航向控制
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
void TurnMotorControl(void)
{
 
	if(quadrupedControl.legAngle >= -46.0f*PI/180.0f && quadrupedControl.legAngle <= 46.0f*PI/180.0f)
	{
		/*正常直线*/
		legOne.legPos.turn=-(quadrupedControl.legAngle+quadrupedControl.yawAdjust);
		legOne.legPos.turn=MaxMinLimit(legOne.legPos.turn,100.0f*PI/180.0f);
		
		legFor.legPos.turn=-(quadrupedControl.legAngle-quadrupedControl.yawAdjust);
		legFor.legPos.turn=MaxMinLimit(legFor.legPos.turn,100.0f*PI/180.0f);
		
		legTwo.legPos.turn=-(quadrupedControl.legAngle+quadrupedControl.yawAdjust);
		legTwo.legPos.turn=MaxMinLimit(legTwo.legPos.turn,100.0f*PI/180.0f);

		legThr.legPos.turn=-(quadrupedControl.legAngle-quadrupedControl.yawAdjust);
		legThr.legPos.turn=MaxMinLimit(legThr.legPos.turn,100.0f*PI/180.0f);
	}
	else if(quadrupedControl.legAngle < -46.0f*PI/180.0f)
	{
		/*红场上坡*/
		legOne.legPos.turn=-(quadrupedControl.legAngle-quadrupedControl.yawAdjust);
		legOne.legPos.turn=MaxMinLimit(legOne.legPos.turn,100.0f*PI/180.0f);
		
		legTwo.legPos.turn=-(quadrupedControl.legAngle+quadrupedControl.yawAdjust);
		legTwo.legPos.turn=MaxMinLimit(legTwo.legPos.turn,100.0f*PI/180.0f);

		legThr.legPos.turn=-(quadrupedControl.legAngle-quadrupedControl.yawAdjust);
		legThr.legPos.turn=MaxMinLimit(legThr.legPos.turn,100.0f*PI/180.0f);
		
		legFor.legPos.turn=-(quadrupedControl.legAngle+quadrupedControl.yawAdjust);
		legFor.legPos.turn=MaxMinLimit(legFor.legPos.turn,100.0f*PI/180.0f);
	}
	else if(quadrupedControl.legAngle > 46.0f*PI/180.0f)
	{
		/*正常直线*/
		legOne.legPos.turn=-(quadrupedControl.legAngle+quadrupedControl.yawAdjust);
		legOne.legPos.turn=MaxMinLimit(legOne.legPos.turn,100.0f*PI/180.0f);
		
		legFor.legPos.turn=-(quadrupedControl.legAngle-quadrupedControl.yawAdjust+legFor.ropeTurn);
		legFor.legPos.turn=MaxMinLimit(legFor.legPos.turn,100.0f*PI/180.0f);
		
		legTwo.legPos.turn=-(quadrupedControl.legAngle-quadrupedControl.yawAdjust);
		legTwo.legPos.turn=MaxMinLimit(legTwo.legPos.turn,100.0f*PI/180.0f);
		
		legThr.legPos.turn=-(quadrupedControl.legAngle+quadrupedControl.yawAdjust);
		legThr.legPos.turn=MaxMinLimit(legThr.legPos.turn,100.0f*PI/180.0f);
	
	}

	PosCrl(CAN2,1,legOne.legPos.turn/2.0f/PI*COUNTS_PER_ROUND*REDUCTION_RATIO*TURN_RATIO,legFor.legPos.turn/2.0f/PI*COUNTS_PER_ROUND*REDUCTION_RATIO*TURN_RATIO);
	PosCrl(CAN2,2,legTwo.legPos.turn/2.0f/PI*COUNTS_PER_ROUND*REDUCTION_RATIO*TURN_RATIO,legThr.legPos.turn/2.0f/PI*COUNTS_PER_ROUND*REDUCTION_RATIO*TURN_RATIO);
}

/** 
  * @brief  肩关节和膝关节控制
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
extern int testTime;
extern uint8_t testFlag;
void HipAndKneeMotorControl(float lfHip,float lfKnee,float rfHip,float rfKnee,float lbHip,float lbKnee,float rbHip,float rbKnee,\
							float lfHipNext,float lfKneeNext,float rfHipNext,float rfKneeNext,float lbHipNext,float lbKneeNext,float rbHipNext,float rbKneeNext)
{
	/*当前要到达的位置*/
	legOne.legPos.hip=lfHip*PI/180.0f;
	legOne.legPos.knee=lfKnee*PI/180.0f;
   
	legTwo.legPos.hip=rfHip*PI/180.0f;
	legTwo.legPos.knee=rfKnee*PI/180.0f;
	
	legThr.legPos.hip=lbHip*PI/180.0f;
	legThr.legPos.knee=lbKnee*PI/180.0f;
	
	legFor.legPos.hip=rbHip*PI/180.0f;
	legFor.legPos.knee=rbKnee*PI/180.0f;
	
	/*下一个要到达的位置*/
	legOne.legNextPos.hip=lfHipNext*PI/180.0f;
	legOne.legNextPos.knee=lfKneeNext*PI/180.0f;
   
	
	legTwo.legNextPos.hip=rfHipNext*PI/180.0f;
	legTwo.legNextPos.knee=rfKneeNext*PI/180.0f;

	
	legThr.legNextPos.hip=lbHipNext*PI/180.0f;
	legThr.legNextPos.knee=lbKneeNext*PI/180.0f;
	
	legFor.legNextPos.hip=rbHipNext*PI/180.0f;
	legFor.legNextPos.knee=rbKneeNext*PI/180.0f;
	
	VelOutput(legOne.legPos.hip,legOne.legNextPos.hip,legOne.legPos.knee,legOne.legNextPos.knee,legOne.ID);
	VelOutput(legTwo.legPos.hip,legTwo.legNextPos.hip,legTwo.legPos.knee,legTwo.legNextPos.knee,legTwo.ID);
	VelOutput(legThr.legPos.hip,legThr.legNextPos.hip,legThr.legPos.knee,legThr.legNextPos.knee,legThr.ID);
	VelOutput(legFor.legPos.hip,legFor.legNextPos.hip,legFor.legPos.knee,legFor.legNextPos.knee,legFor.ID);
	
}
/** 
  * @brief  斜向走
  * @note
  * @param  None
  * @retval None
  */
void Oblique(float angle)
{
	if(legOne.standFlag && legFor.standFlag)
	{
		if(legOne.read.y > -15.0f && legOne.read.y < 15.0f)
		{
			quadrupedControl.legAngle=angle;
		}	
	}
	else if(legTwo.standFlag && legThr.standFlag)
	{
		
		if(legTwo.read.y > -15.0f && legTwo.read.y < 15.0f)
		{
			quadrupedControl.legAngle=angle;
		}
		
	}
	
}

/** 
  * @brief  速度控制
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
uint16_t SpeedControl(uint16_t desirePeriod,int16_t acc,uint16_t timeCnt)
{
	uint16_t newPeriod=0;
	
	newPeriod=timeCnt-acc;
	
	if((acc > 0.0f && newPeriod <= desirePeriod) || (acc < 0.0f && newPeriod >= desirePeriod))
		newPeriod=desirePeriod;
	
	if(newPeriod > 300.0f)	
		newPeriod=300.0f;
	if(newPeriod < 65.0f)
		newPeriod=65.0f;
	
	return newPeriod;
}

/** 
  * @brief  期望角变到x轴角度
  * @note
  * @param  dAngle：期望角度，为弧度制
  * @param  None
  * @retval None
  */
float angleTransToX(float dAngle)
{
	float angle=0,angleD=0;
	angleD=dAngle;
	if(angleD > 14.036f*PI/180.0f)
		angleD=14.036f*PI/180.0f;
	else if(angleD < -14.036f*PI/180.0f)
		angleD=-14.036f*PI/180.0f;
		
	angle=atanf(-tanf(angleD)/sqrtf(2.0f))/PI*180.0f;

	return angle;
}

/** 
  * @brief  期望角变到y轴角度
  * @note
  * @param  dAngle：期望角度，为弧度制
  * @param  None
  * @retval None
  */
float angleTransToY(float dAngle)
{
	float angle=0,angleD=0;
	angleD=dAngle;
	if(angleD > 14.036f*PI/180.0f)
		angleD=14.036f*PI/180.0f;
	else if(angleD < -14.036f*PI/180.0f)
		angleD=-14.036f*PI/180.0f;
	angle=asinf(sinf(angleD)/sqrtf(2.0f))/PI*180.0f;
	
	return angle;
}



/*****************************************************************/










