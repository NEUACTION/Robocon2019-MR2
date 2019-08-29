/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "moveBase.h"
#include "sensor.h"
#include "errorHand.h"
#include "balance.h"
#include "cloudplatform.h"
#include "switch.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
uint16_t timeOut=0;
uint16_t timeOut2=0;
extern leg_ legOne, legTwo, legThr, legFor;  

extern BalancerType gBalancer1;
extern BalancerType gBalancer2;
extern reset_ Reset;


//转换CAN接收的数据
union Translate
{
	int32_t data32;
	uint8_t data8[4];

}translation;
//void CAN1_RX0_IRQHandler(void)
//{
//	OS_CPU_SR cpu_sr;

//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();
//	
//	int32_t getPos=0,getVel=0;
//	
//	//CAN1口接受任意数据以通过标志位中断
//	uint32_t StdId = 0x00;
//	uint8_t CAN1Buffer1[8] = {0};
//	uint8_t receiveLength = 8;
//	
//    CAN_RxMsg(CAN1, &StdId, CAN1Buffer1, &receiveLength);

//	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
//	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
//	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
//	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);
//	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
//	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
//	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
//	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
//	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
//	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
//	OSIntExit();
//}

/**
  * @brief  CAN2 receive FIFO0 interrupt request handler
  * @note
  * @param  None
  * @retval None
  */
extern uint8_t canSendFlag;  
extern int can2ErrNum[4];
extern float CPPos;
void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	//CAN2口接受任意数据以通过标志位中断
	uint32_t StdId = 0x00;
	uint8_t CAN2Buffer1[4] = {0};
	uint8_t receiveLength = 4;
	
    CAN_RxMsg(CAN2, &StdId, CAN2Buffer1, &receiveLength);
	
	for(uint8_t i=0;i<=3;i++)
		translation.data8[i]=CAN2Buffer1[i];
    
    /*读4个2006航向电机数据以及3508平衡陀螺的数据*/
	switch (StdId)
	{
		case 0x221:
			legOne.readPos.turn =-translation.data32/COUNTS_PER_ROUND/REDUCTION_RATIO/TURN_RATIO*2.0f*PI;
			can2ErrNum[0]-=2;
			break;
		case 0x222:
			legTwo.readPos.turn =-translation.data32/COUNTS_PER_ROUND/REDUCTION_RATIO/TURN_RATIO*2.0f*PI;
			can2ErrNum[1]-=2;
			break;
		case 0x223:
			legThr.readPos.turn =-translation.data32/COUNTS_PER_ROUND/REDUCTION_RATIO/TURN_RATIO*2.0f*PI;
			can2ErrNum[2]-=2;
			break;		
		case 0x224:
			legFor.readPos.turn =-translation.data32/COUNTS_PER_ROUND/REDUCTION_RATIO/TURN_RATIO*2.0f*PI;
			can2ErrNum[3]-=2;
			break;
		case 0x225:
			Reset.liftPos=translation.data32;
			break;
        case 0x226:
			gBalancer1.motor.posPulse = translation.data32;                              //平衡陀螺3508电机的位置
			gBalancer1.motor.pos = PULSE_TO_BALANCER_DEGREE(gBalancer1.motor.posPulse);     //转换为陀螺的进动角/度
			break;
		case 0x227:
			gBalancer2.motor.posPulse = translation.data32;                              //平衡陀螺3508电机的位置
			gBalancer2.motor.pos = PULSE_TO_BALANCER_DEGREE(gBalancer2.motor.posPulse);     //转换为陀螺的进动角/度
			break;
		case 0x206:
			gBalancer1.motor.speedPulse = translation.data32;                             //平衡陀螺3508电机的速度   	
			gBalancer1.motor.speed= PULSE_TO_BALANCER_DEGREE(gBalancer1.motor.speedPulse);   //平衡陀螺3508电机的速度
			break;
        case 0x207:
			gBalancer2.motor.speedPulse = translation.data32;	
			gBalancer2.motor.speed= PULSE_TO_BALANCER_DEGREE(gBalancer2.motor.speedPulse);
			break;
            
		default:break;
	
	}	
	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次
#define PERIOD_COUNTER PERIOD 

extern uint16_t timeRecord;

extern OS_EVENT *PeriodSem;
extern OS_EVENT *BalancePeriodSem;

extern uint8_t protectFlag;
extern uint16_t timeCount;

#define BalanceControlPeriod 50
uint8_t timeCount2=BalanceControlPeriod;

void TIM2_IRQHandler(void)
{
	//用来计数100次，产生10ms的定时器
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		
		timeCount--;
		if (timeCount == 0)
		{
			OSSemPost(PeriodSem);
			timeCount = timeRecord;
		}
        
      
        timeCount2--;
        if(timeCount2==0)
        {
            OSSemPost(BalancePeriodSem);
			timeCount2 = BalanceControlPeriod;
		}
       

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}

  

void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM8_UP_TIM13_IRQHandler(void) 
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}
 
int testTime=0;
uint8_t testFlag=0;
extern uint8_t gyroFlag;
#define INIT_PERIOD_COUNTER	150
extern OS_EVENT *ConfigPeriodSem;
void TIM3_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	static int gyroWaitCnt=0;
	static uint8_t configPeriodCounter=INIT_PERIOD_COUNTER;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		timeOut++;
		timeOut2++;
		if(gyroFlag == 1)
			gyroWaitCnt++;
		
		/*陀螺转20秒*/
		if(gyroWaitCnt > 220000)
		{
			gyroWaitCnt=0;
			gyroFlag=2;
		}
		
		if(testFlag == 1)
			testTime++;
			
		configPeriodCounter--;
		if(configPeriodCounter == 0)
		{
			
			OSSemPost(ConfigPeriodSem);
			configPeriodCounter=INIT_PERIOD_COUNTER;
		}
        
        
        
        
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}

void UART4_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART4, USART_IT_RXNE) == SET)
	{

		USART_ClearITPendingBit(UART4, USART_IT_RXNE);
	}
	OSIntExit();
}
/***************************试场调参数用蓝牙串口中断*****************************************************/
void USART1_IRQHandler(void)
{

	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
	OSIntExit();
}

//void USART2_IRQHandler(void)
//{
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();

//	if (USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
//	}
//	OSIntExit();
//}

//void USART6_IRQHandler(void) //更新频率200Hz
//{
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();

//	if (USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART6, USART_IT_RXNE);
//		
//	}
//	else
//	{
//		USART_ClearITPendingBit(USART6, USART_IT_PE);
//		USART_ClearITPendingBit(USART6, USART_IT_TXE);
//		USART_ClearITPendingBit(USART6, USART_IT_TC);
//		USART_ClearITPendingBit(USART6, USART_IT_ORE_RX);
//		USART_ClearITPendingBit(USART6, USART_IT_IDLE);
//		USART_ClearITPendingBit(USART6, USART_IT_LBD);
//		USART_ClearITPendingBit(USART6, USART_IT_CTS); 
//		USART_ClearITPendingBit(USART6, USART_IT_ERR);
//		USART_ClearITPendingBit(USART6, USART_IT_ORE_ER);
//		USART_ClearITPendingBit(USART6, USART_IT_NE);
//		USART_ClearITPendingBit(USART6, USART_IT_FE);
//		USART_ReceiveData(USART6);
//	}
//	OSIntExit();
//}

//void USART3_IRQHandler(void)
//{
//	OS_CPU_SR cpu_sr;
//	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
//	OSIntNesting++;
//	OS_EXIT_CRITICAL();

//	if (USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
//	{
//		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
//	}

//	OSIntExit();
//}


void UART5_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if (USART_GetITStatus(UART5, USART_IT_RXNE) == SET)
	{
		USART_ClearITPendingBit(UART5, USART_IT_RXNE);
	}
	else
	{
		USART_ClearITPendingBit(UART5, USART_IT_PE);
		USART_ClearITPendingBit(UART5, USART_IT_TXE);
		USART_ClearITPendingBit(UART5, USART_IT_TC);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_RX);
		USART_ClearITPendingBit(UART5, USART_IT_IDLE);
		USART_ClearITPendingBit(UART5, USART_IT_LBD);
		USART_ClearITPendingBit(UART5, USART_IT_CTS);
		USART_ClearITPendingBit(UART5, USART_IT_ERR);
		USART_ClearITPendingBit(UART5, USART_IT_ORE_ER);
		USART_ClearITPendingBit(UART5, USART_IT_NE);
		USART_ClearITPendingBit(UART5, USART_IT_FE);
		USART_ReceiveData(UART5);
	}
	OSIntExit();
}

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	while (1)
	{
		
	}
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{

	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
		
		USART_OUT(UART4,(uint8_t *)" HardFault");
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{

	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
