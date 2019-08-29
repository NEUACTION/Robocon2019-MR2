/**
*****************************************
错误判断并处理
*****************************************
*/

#include "errorHand.h"
#include "light.h"
#include "gpio.h"
/*can1上检测肩关节和膝关节是否正常变量*/
int can1ErrNum[8]={0};

/*can2上检测航向是否正常变量*/
int can2ErrNum[4]={0};


extern leg_ legOne, legTwo, legThr, legFor;  

/** 
  * @brief  can通讯错误检测
  * @note
  * @param  None
  * @retval None
  */
uint8_t protectFlag=0;
uint8_t CanCommunicateErr(void)
{
	

	
	if(can1ErrNum[0] < 0)
		can1ErrNum[0]=0;
	if(can1ErrNum[1] < 0)
		can1ErrNum[1]=0;
	if(can1ErrNum[2] < 0)
		can1ErrNum[2]=0;
	if(can1ErrNum[3] < 0)
		can1ErrNum[3]=0;
	if(can1ErrNum[4] < 0)
		can1ErrNum[4]=0;
	if(can1ErrNum[5] < 0)
		can1ErrNum[5]=0;
	if(can1ErrNum[6] < 0)
		can1ErrNum[6]=0;
	if(can1ErrNum[7] < 0)
		can1ErrNum[7]=0;
	
	if(can2ErrNum[0] < 0)
		can2ErrNum[0]=0;
	if(can2ErrNum[1] < 0)
		can2ErrNum[1]=0;
	if(can2ErrNum[2] < 0)
		can2ErrNum[2]=0;
	if(can2ErrNum[3] < 0)
		can2ErrNum[3]=0;
	
	/*累计有15次读不到电机位置*/
	if(can1ErrNum[0] > 10 || can1ErrNum[1] > 10 || can1ErrNum[2] > 10 || can1ErrNum[3] > 10\
	|| can1ErrNum[4] > 10 || can1ErrNum[5] > 10 || can1ErrNum[6] > 10 || can1ErrNum[7] > 10\
	|| can2ErrNum[0] > 10 || can2ErrNum[1] > 10 || can2ErrNum[2] > 10 || can2ErrNum[3] > 10)
		protectFlag =1;
	
	return protectFlag;
	
}

/** 
  * @brief  can通讯错误显示
  * @note
  * @param  None
  * @retval None
  */
void CanErrDisplay(void)
{
	/*1号腿肩关节电机can读不到，亮绿灯，如果膝关节电机同时读不到则亮白灯，其他腿一样*/		
	if(can1ErrNum[0] > 10)
	{
		if(can1ErrNum[1] > 10)
		{
			White(LEGONE_LIGHT_ID);
			USART_OUT(UART4,(uint8_t *)"  legOne hip and knee is lost\t\t");
		}
		else
		{
			Green(LEGONE_LIGHT_ID);
			USART_OUT(UART4,(uint8_t *)"  legOne hip is lost\t\t");
		}
		
	}
	
	/*只有膝关节电机can读不到亮金色灯*/
	else if(can1ErrNum[1] > 10)
	{
		Gold(LEGONE_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legOne knee is lost\t\t");
	}
	
	/*2号腿can出问题*/
	if(can1ErrNum[2] > 10)
	{
		if(can1ErrNum[3] > 10)
		{
			USART_OUT(UART4,(uint8_t *)"  legTwo hip and knee is lost\t\t");
			White(LEGTWO_LIGHT_ID);
		}
		else
		{
			USART_OUT(UART4,(uint8_t *)"  legTwo hip is lost\t\t");
			Green(LEGTWO_LIGHT_ID);
		}
		
	}
	else if(can1ErrNum[3] > 10)
	{
		Gold(LEGTWO_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legTwo knee is lost\t\t");
	}
	
	/*3号腿can出问题*/
	if(can1ErrNum[4] > 10)
	{
		if(can1ErrNum[3] > 10)
		{
			USART_OUT(UART4,(uint8_t *)"  legThr hip and knee is lost\t\t");
			White(LEGTHR_LIGHT_ID);
		}
		else
		{
			USART_OUT(UART4,(uint8_t *)"  legThr hip is lost\t\t");
			Green(LEGTHR_LIGHT_ID);
		}
	}
	else if(can1ErrNum[5] > 10)
	{
		Gold(LEGTHR_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legThr knee is lost\t\t");
	}
	
	/*4号腿can出问题*/
	if(can1ErrNum[6] > 10)
	{
		if(can1ErrNum[3] > 10)
		{
			USART_OUT(UART4,(uint8_t *)"  legFor hip and knee is lost\t\t");
			White(LEGFOR_LIGHT_ID);
		}
		else
		{
			USART_OUT(UART4,(uint8_t *)"  legFor hip is lost\t\t");
			Green(LEGFOR_LIGHT_ID);
		}
	}
	else if(can1ErrNum[7] > 10)
	{
		Gold(LEGFOR_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legFor knee is lost\t\t");
	}
	
	
	/*航向电机can读不到亮黄绿灯*/
	if(can2ErrNum[0] > 10)
	{
		Kelly(LEGONE_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legOne turn is lost\t\t");
	}
	
	if(can2ErrNum[1] > 10)
	{
		Kelly(LEGTWO_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legTwo turn is lost\t\t");
	}
	
	if(can2ErrNum[2] > 10)
	{
		Kelly(LEGTHR_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legThr turn is lost\t\t");
	}
	
	if(can2ErrNum[3] > 10)
	{
		Kelly(LEGFOR_LIGHT_ID);
		USART_OUT(UART4,(uint8_t *)"  legFor turn is lost\t\t");
	}
	
	VelCrl(CAN1,legOne.ID,0,0);
    VelCrl(CAN1,legTwo.ID,0,0);
    VelCrl(CAN1,legThr.ID,0,0);
    VelCrl(CAN1,legFor.ID,0,0);
    
    VelCrl2(CAN2,GYRO1,0); 
    VelCrl2(CAN2,GYRO2,0);

}


/** 
  * @brief  角度太大保护
  * @note
  * @param  None
  * @retval None
  */
uint8_t out=0;
uint8_t RolloverProtect(float *euler)
{
    extern uint16_t gaitCnt;
	if(euler[0] >  25.0f*PI/180.0f ||  euler[0] <  -25.0f*PI/180.0f || euler[1] > 25.0f*PI/180.0f || euler[1] < -25.0f*PI/180.0f || euler[2] > 15.0f*PI/180.0f || euler[2] < -15.0f*PI/180.0f)
		out=1;
	if(gaitCnt==30)
        out=0;
   
	return out;

}


uint8_t tokenLost=0;
uint8_t TokenErr(void)
{
    static uint16_t tokenErrCnt=0;
    
    uint8_t tokenFlag=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_10);
    /*未挡住为1，挡住为0*/
    if(tokenFlag==1)
        tokenErrCnt++;
    else tokenErrCnt=0;
    
    if(tokenErrCnt>10)
    {
        tokenLost=1;
        tokenErrCnt=0;
    }
    
    return tokenLost;
}











