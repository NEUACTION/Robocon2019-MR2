/**
*****************************************
四足机器人运动总过程
各个模块在module.c中
*****************************************
*/
/* Includes -------------------------------------------------------------------------------------------*/

#include "moveBase.h"
#include "sensor.h"
#include "module_RED.h"
#include "module_BLUE.h"
#include "vloop.h"
#include "light.h"
#include "pid.h"
#include "errorHand.h"
#include "switch.h"
/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/**
  * @brief  
  * @note
  * @param  
  * @retval None
  */
/**************全局变量*******************/

/*四足腿部控制以及状态结构体*/
leg_ legOne, legTwo, legThr, legFor;  

/*四足整体控制结构体*/
quadrupedControl_ quadrupedControl;

/*四足整体状态结构体*/
quadrupedStatus_ quadrupedStatus;

/*机器人运动过程所处阶段变量*/
uint8_t stage=START;

/***************extern********************/
extern visual_ visual;
extern uint16_t gaitCnt;
extern uint8_t closeLoopFlag;
/** 
  * @brief  红场四足运动过程
  * @note
  * @param  None
  * @retval None
  */
uint8_t changFlag=0;
void walk_RED(void)
{
	switch(stage)
	{
		
		case START:
			changFlag=StartUp_RED();
			if(changFlag == 1)
			{
				stage=FLAT;
				changFlag=0;
			}
			break;
		case FLAT:
			changFlag=Stright_RED();
			if(changFlag == 1)
			{
				stage=STEP;
				changFlag=0;
			}
			break;
	
		case STEP:
			changFlag=StepOver_RED();
			if(changFlag == 1)
			{
				stage= STEP_TO_ROPE;
				changFlag=0;
			}
			break;	
		case STEP_TO_ROPE:
			changFlag=STRTrans_RED();
			if(changFlag == 1)
			{
				stage=ROPE;
				changFlag=0;
			}
			break;
		case ROPE:
			changFlag=CrossRope_RED();
			if(changFlag == 1)
			{
                stage=RELAY;
				changFlag=0;
			}
			break;
		case RELAY:
			changFlag=Relay_RED();
			if(changFlag == 1)
			{
				stage=UPHILL;
				changFlag=0;
			}
			break;
		case UPHILL:
			changFlag=Uphill_RED();
			if(changFlag == 1)
			{
				stage=STOP;
				changFlag=0;
			}
			break;
		case STOP:
			
			/*闭环停止调节*/
			closeLoopFlag=0;
		
			VelOutput(legOne.legPos.hip,legOne.legPos.hip,legOne.legPos.knee,legOne.legPos.knee,legOne.ID);
			VelOutput(legTwo.legPos.hip,legTwo.legPos.hip,legTwo.legPos.knee,legTwo.legPos.knee,legTwo.ID);
			VelOutput(legThr.legPos.hip,legThr.legPos.hip,legThr.legPos.knee,legThr.legPos.knee,legThr.ID);
			VelOutput(legFor.legPos.hip,legFor.legPos.hip,legFor.legPos.knee,legFor.legPos.knee,legFor.ID);
			break;
		default:
			break;
	}
	
}
/** 
* @brief  蓝场四足运动过程
  * @note
  * @param  None
  * @retval None
  */
void walk_BLUE(void)
{

	switch(stage)
	{
		case START:
			changFlag=StartUp_BLUE();
			if(changFlag == 1)
			{
				stage=FLAT;
				changFlag=0;
			}
			break;
		case FLAT:
			changFlag=Stright_BLUE();
			if(changFlag == 1)
			{
				stage=STEP;
				changFlag=0;
			}
			break;
	
		case STEP:
			changFlag=StepOver_BLUE();
			if(changFlag == 1)
			{
				stage=STEP_TO_ROPE;
				changFlag=0;
			}
			break;	
		case STEP_TO_ROPE:
			changFlag=STRTrans_BLUE();
			if(changFlag == 1)
			{
				stage=ROPE;
				changFlag=0;
			}
			break;
		case ROPE:
			changFlag=CrossRope_BLUE();
			if(changFlag == 1)
			{

                stage=RELAY;
				changFlag=0;
			}
			break;
		case RELAY:
			changFlag=Relay_BLUE();
			if(changFlag == 1)
			{
				stage=UPHILL;
				changFlag=0;
			} 
			break;
		case UPHILL:
			changFlag=Uphill_BLUE();
			if(changFlag == 1)
			{
				stage=STOP;
				changFlag=0;
			}
			break;
		case STOP:
			
			/*闭环停止调节*/
			closeLoopFlag=0;
		
			VelOutput(legOne.legPos.hip,legOne.legPos.hip,legOne.legPos.knee,legOne.legPos.knee,legOne.ID);
			VelOutput(legTwo.legPos.hip,legTwo.legPos.hip,legTwo.legPos.knee,legTwo.legPos.knee,legTwo.ID);
			VelOutput(legThr.legPos.hip,legThr.legPos.hip,legThr.legPos.knee,legThr.legPos.knee,legThr.ID);
			VelOutput(legFor.legPos.hip,legFor.legPos.hip,legFor.legPos.knee,legFor.legPos.knee,legFor.ID);
			break;
		default:
			break;
	}
	
}

/**

  * @brief  速度规划，改变程序周期改变速度
  * @note
  * @param  None
  * @retval None
  */
extern uint8_t site;
extern float desireAngle_X,desireAngle_Y;
int16_t timeAdd=0;
int8_t STRacc=2; 
void SpeedPlan(uint16_t *timeLast,uint16_t *timeCnt)
{
	extern uint8_t slowFlag;
    
	uint8_t slowSpeed=0;
	
	
	if(site == RED)
	{
		if(slowFlag==1)
		slowSpeed=10;
		else slowSpeed=0;
		timeAdd=timeControl(0.4f*PI/180.0f,1.1f*PI/180.0f);
		/*加减速*/
		if(gaitCnt < 2)
			(*timeCnt)=SpeedControl(70,2,(*timeLast));//12
		else if(gaitCnt < 12)
		{
			if(timeAdd >= 0)
				(*timeCnt)=SpeedControl(70+timeAdd,-1,(*timeLast));//12
			else
				(*timeCnt)=SpeedControl(70+timeAdd,1,(*timeLast));//12
		}
		else if(gaitCnt>=12&&gaitCnt<15)
			(*timeCnt)=SpeedControl(100,-1,(*timeLast));//13
		else if(gaitCnt>=15&&gaitCnt<18)
			(*timeCnt)=SpeedControl(100,-1,(*timeLast));//13
		else if(gaitCnt>=18&&gaitCnt<21)
            {
                if(STRacc==2||gaitCnt==18)
                    (*timeCnt)=SpeedControl(70,2,(*timeLast));//12
                if(STRacc==1)
                {
                    if(gaitCnt==19)
                        (*timeCnt)=SpeedControl(150,1,(*timeLast));//12
                    if(gaitCnt==20)
                        (*timeCnt)=SpeedControl(120,2,(*timeLast));//12
                }
            }
		else if(gaitCnt >= 21 && gaitCnt < 30)
        {
			if(STRacc==2)
                (*timeCnt)=SpeedControl(110,-1,(*timeLast));//17
            if(STRacc==1)
                (*timeCnt)=SpeedControl(120,-1,(*timeLast));//17
        }
        else if(gaitCnt >= 30 && gaitCnt < 32)
			(*timeCnt)=SpeedControl(140+slowSpeed,-1,(*timeLast));//200
        else if(gaitCnt >= 32 && gaitCnt < 34)
			(*timeCnt)=SpeedControl(95+slowSpeed,3,(*timeLast));//170
        else if(gaitCnt >= 34 && gaitCnt < 40)
			(*timeCnt)=SpeedControl(95+slowSpeed,1,(*timeLast));//140
		else if(gaitCnt >= 40 && gaitCnt < 43)
			(*timeCnt)=SpeedControl(95+slowSpeed ,-1,(*timeLast));//170
		else if(gaitCnt >= 43)
			(*timeCnt)=SpeedControl(90+slowSpeed ,-1,(*timeLast));//170
	}
	else
	{
		if(slowFlag==1)
		slowSpeed=0;
		else slowSpeed=0;
		
		timeAdd=timeControl(0.4f*PI/180.0f,1.2f*PI/180.0f);
		/*加减速*/
		if(gaitCnt < 2)
			(*timeCnt)=SpeedControl(70,2,(*timeLast));//12
		else if(gaitCnt < 12)
		{
			if(timeAdd >= 0)
				(*timeCnt)=SpeedControl(70+timeAdd,-1,(*timeLast));//12
			else
				(*timeCnt)=SpeedControl(70+timeAdd,1,(*timeLast));//12
		}
		else if(gaitCnt>=12&&gaitCnt<15)
			(*timeCnt)=SpeedControl(100,-1,(*timeLast));//13
		else if(gaitCnt>=15&&gaitCnt<18)
			(*timeCnt)=SpeedControl(100,-1,(*timeLast));//13
		else if(gaitCnt>=18&&gaitCnt<21)
            {
                if(STRacc==2||gaitCnt==18)
                    (*timeCnt)=SpeedControl(70,2,(*timeLast));//12
                if(STRacc==1)
                {
                    if(gaitCnt==19)
                        (*timeCnt)=SpeedControl(150,1,(*timeLast));//12
                    if(gaitCnt==20)
                        (*timeCnt)=SpeedControl(120,2,(*timeLast));//12
                }
            }
		else if(gaitCnt >= 21 && gaitCnt < 30)
        {
			if(STRacc==2)
                (*timeCnt)=SpeedControl(110,-1,(*timeLast));//17
            if(STRacc==1)
                (*timeCnt)=SpeedControl(120,-1,(*timeLast));//17
        }
        else if(gaitCnt >= 30 && gaitCnt < 32)
			(*timeCnt)=SpeedControl(140+slowSpeed,-1,(*timeLast));//200
        else if(gaitCnt >= 32 && gaitCnt < 34)
			(*timeCnt)=SpeedControl(100+slowSpeed,3,(*timeLast));//170
        else if(gaitCnt >= 34 && gaitCnt < 40)
			(*timeCnt)=SpeedControl(100+slowSpeed,1,(*timeLast));//140
		else if(gaitCnt >= 40 && gaitCnt < 43)
			(*timeCnt)=SpeedControl(100+slowSpeed ,-1,(*timeLast));//170
		else if(gaitCnt >= 43)
			(*timeCnt)=SpeedControl(100+slowSpeed ,-1,(*timeLast));//170
	}
		
	
	(*timeLast)=(*timeCnt);
	
}

/**
  * @brief  平方运算
  * @note
  * @param  num：平方运算输入参数
  * @retval None
  */
float Cubed(float num)
{
	float out = 0;
	out = num * num*num;
	return out;
}

/**
  * @brief  立方运算
  * @note
  * @param  num：立方运算输入参数
  * @retval None
  */
float Square(float num)
{
	float out = 0;
	out = num * num;
	return out;
}

/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
