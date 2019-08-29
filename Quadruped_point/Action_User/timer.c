/**
  ******************************************************************************
  * @file    timer.c
  * @author  Calcus Lee
  * @version V1.0.1
  * @date    9-August-2013
  * @brief   functions of time
  ******************************************************************************
  * @attention
  * three are some great and accurency time delay function in timer.h
  *
**/

#include "timer.h"
#include "misc.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

//精确延时函数调用wait
void wait(uint32_t n)
{
	do
	{
		n--;
	} while (n);
}

/**
  * @brief  Configures the TIMx's interrupt time.
  * @param  TIMx: where x can be 1-14. 
			@note TIM1 and TIM 9,10,11;TIM2 and TIM12,13,14 have some sharing IRQ handle functions.
  * @param  arr: the period value to be loaded into the active
                 Auto-Reload Register at the next update event, range from 1 to 65535
  * @param  psr: the prescaler value used to divide the TIM clock, range from 1 to 65535
  * @note   1.the TIMx's PreemptionPriority and SubPriority has been predefined in this function,
			change them according to your need.
			2.Tout= ((arr+1)*(psc+1))/Tclk s, 
			  for TIM2-7 and TIM12-14, Tclk=84M
			  for TIM1,8,9,10,11 Tclk=168M
			  exp: for TIM2, if arr=999, psc=83, then Tout=(1000*84)/84M=1ms
  * @retval None
  * @author Calcus Lee
  */
void TIM_Init(TIM_TypeDef *TIMx, uint16_t arr, uint16_t psr, uint16_t prepri, uint16_t subpri)
{
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	switch ((uint32_t)TIMx)
	{
	//APB2 TIM
	case TIM1_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn;

		break;
	}
	case TIM8_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;

		break;
	}
	case TIM9_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_TIM9_IRQn;

		break;
	}
	case TIM10_BASE:
	{
		RCC_APB2PeriphClockCmd(TIM1_UP_TIM10_IRQn, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;

		break;
	}
	case TIM11_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM1_TRG_COM_TIM11_IRQn;

		break;
	}
	//APB1 TIM
	case TIM2_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;

		break;
	}
	case TIM3_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;

		break;
	}
	case TIM4_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;

		break;
	}
	case TIM5_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;

		break;
	}
	case TIM6_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;

		break;
	}
	case TIM7_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;

		break;
	}
	case TIM12_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;

		break;
	}
	case TIM13_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;

		break;
	}
	case TIM14_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
		NVIC_InitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;

		break;
	}

	default:
		break;
	}

	//定时器TIMx初始化
	TIMx_TimeBaseStructure.TIM_Period = arr;					 //设置自动重转载寄存器周期的值
	TIMx_TimeBaseStructure.TIM_Prescaler = psr;					 //设置时钟分频除数的预分频值
	TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 //设置时钟分割
	TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);			 //初始化TIMx
	TIM_ClearITPendingBit(TIMx, TIM_IT_Update);					 //初始化时必须将溢出中断清0,必须在开溢出中断之前
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);					 //允许溢出中断
	//中断优先级NVIC设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = prepri; //抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = subpri;		   //从优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				   //使能IRQ通道
	NVIC_Init(&NVIC_InitStructure);								   //初始化NVIC寄存器

	TIM_Cmd(TIMx, ENABLE); //使能TIMx
}

/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs: n us you want to delay, range from 1 to 65535
			@note	TIM2 and TIM5 are 32bit timer, you may change the code to delay longer;
					every time when time= 168/168M s,TIMCounter++ 
  * @retval None
  * @authonr Calcus Lee
  */
void TIM_Delayus(TIM_TypeDef *TIMx, uint16_t Delayus)
{
	uint16_t TIMCounter = Delayus;
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;

	switch ((uint32_t)TIMx)
	{
	//APB2 TIM
	case TIM1_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 168; //设置时钟分频除数的预分频值

		break;
	}
	case TIM8_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 168; //设置时钟分频除数的预分频值

		break;
	}
	case TIM9_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 168; //设置时钟分频除数的预分频值

		break;
	}
	case TIM10_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 168; //设置时钟分频除数的预分频值

		break;
	}
	case TIM11_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 168; //设置时钟分频除数的预分频值

		break;
	}
	//APB1 TIM
	case TIM2_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM3_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM4_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM5_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM6_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM7_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM12_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM13_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	case TIM14_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 84; //设置时钟分频除数的预分频值

		break;
	}
	default:
		break;
	}

	TIMx_TimeBaseStructure.TIM_Period = 1;						 //设置自动重转载寄存器周期的值
	TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 //设置时钟分割
	TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);			 //初始化TIM1

	TIM_Cmd(TIMx, ENABLE);
	TIM_SetCounter(TIMx, 65535 - TIMCounter);

	while (TIMCounter < 65535)
	{
		TIMCounter = TIM_GetCounter(TIMx);
	}

	TIM_Cmd(TIMx, DISABLE);
}

/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs: n 100us you want to delay, range from 1 to 65535
			@note	TIM2 and TIM5 are 32bit timer, you may change the code to delay longer
  * @retval None
  * @authonr Calcus Lee
  */
void TIM_Delay100us(TIM_TypeDef *TIMx, uint16_t Delay100us)
{
	uint16_t TIMCounter = Delay100us;
	TIM_TimeBaseInitTypeDef TIMx_TimeBaseStructure;

	switch ((uint32_t)TIMx)
	{
	//APB2 TIM
	case TIM1_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 16800; //设置时钟分频除数的预分频值

		break;
	}
	case TIM8_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 16800; //设置时钟分频除数的预分频值

		break;
	}
	case TIM9_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 16800; //设置时钟分频除数的预分频值

		break;
	}
	case TIM10_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 16800; //设置时钟分频除数的预分频值

		break;
	}
	case TIM11_BASE:
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 16800; //设置时钟分频除数的预分频值

		break;
	}
	//APB1 TIM
	case TIM2_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM3_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM4_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM5_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM6_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM7_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM12_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM13_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	case TIM14_BASE:
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
		TIMx_TimeBaseStructure.TIM_Prescaler = 8400; //设置时钟分频除数的预分频值

		break;
	}
	default:
		break;
	}

	TIMx_TimeBaseStructure.TIM_Period = 1;						 //设置自动重转载寄存器周期的值
	TIMx_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	 //设置时钟分割
	TIMx_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM向上计数
	TIM_TimeBaseInit(TIMx, &TIMx_TimeBaseStructure);			 //初始化TIM1

	TIM_Cmd(TIMx, ENABLE);
	TIM_SetCounter(TIMx, 65535 - TIMCounter);

	while (TIMCounter < 65535)
	{
		TIMCounter = TIM_GetCounter(TIMx);
	}

	TIM_Cmd(TIMx, DISABLE);
}

/**
  * @brief  accurency time delay dunction with TIMx
  * @param  TIMx: where x can be 1-14. 
  * @param  DelayMs:
  * @retval None
  * @authonr Calcus Lee
  */
void TIM_Delayms(TIM_TypeDef *TIMx, uint32_t DelayMs)
{
	uint32_t i = 0;

	for (i = 0; i < DelayMs; i++)
	{
		TIM_Delay100us(TIMx, 10);
	}
}

void TIM3_Pwm_Init(u32 arr, u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  //TIM4 时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //使能 PORTB 时钟

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3); //PB7 复用为 TIM4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;		   //GPIOB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //速度 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);			   //初始化 PB7

	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;						//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化定时器 4

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //PWM 调制模式 2

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 //输出极性低
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);					  //初始化外设 TIM1 4OC1
	TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);			  //使能预装载寄存器
	TIM_ARRPreloadConfig(TIM3, ENABLE);							  //ARPE 使能
	TIM_Cmd(TIM3, ENABLE);										  //使能 TIM4
	TIM_SetCompare2(TIM3, 0.05 * 2000);
}

void TIM4_Pwm_Init(u32 arr, u32 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  //TIM4 时钟使能
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //使能 PORTD 时钟

	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); //复用为 TIM4
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource14, GPIO_AF_TIM4); //复用为 TIM4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_14; //GPIOB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			 //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;		 //速度 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			 //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			 //上拉
	GPIO_Init(GPIOD, &GPIO_InitStructure);					 //初始化 PB7

	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period = arr;						//自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //初始化定时器 4

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;			  //PWM 调制模式 2
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;	 //输出极性低
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);					  //初始化外设 TIM4 OC1
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);					  //初始化外设 TIM4 OC3

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); //使能预装载寄存器
	TIM_ARRPreloadConfig(TIM4, ENABLE);				  //ARPE 使能
	TIM_Cmd(TIM4, ENABLE);							  //使能 TIM4

	TIM_SetCompare1(TIM4, 0.05 * 2000);
	TIM_SetCompare3(TIM4, 0.05 * 2000);
}

/*********************************WIFI*************************/
/**************************************************************/

extern vu16 USART3_RX_STA;

//通用定时器中断初始化
//这里始终选择为APB1的2倍，而APB1为36M
//arr：自动重装值。
//psc：时钟预分频数
void TIM7_Int_Init(u16 arr, u16 psc)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE); //TIM7鏃堕挓浣胯兘

	//瀹氭椂鍣═IM7鍒濆鍖?
	TIM_TimeBaseStructure.TIM_Period = arr;						//璁剧疆鍦ㄤ笅涓€涓洿鏂颁簨浠惰鍏ユ椿鍔ㄧ殑鑷姩閲嶈杞藉瘎瀛樺櫒鍛ㄦ湡鐨勫€?
	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//璁剧疆鐢ㄦ潵浣滀负TIMx鏃堕挓棰戠巼闄ゆ暟鐨勯鍒嗛鍊?
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;		//璁剧疆鏃堕挓鍒嗗壊:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM鍚戜笂璁℃暟妯″紡
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);				//鏍规嵁鎸囧畾鐨勫弬鏁板垵濮嬪寲TIMx鐨勬椂闂村熀鏁板崟浣?

	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); //浣胯兘鎸囧畾鐨凾IM7涓柇,鍏佽鏇存柊涓柇

	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; //鎶㈠崰浼樺厛绾?
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		  //瀛愪紭鍏堢骇1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  //IRQ閫氶亾浣胯兘
	NVIC_Init(&NVIC_InitStructure);							  //鏍规嵁鎸囧畾鐨勫弬鏁板垵濮嬪寲VIC瀵勫瓨鍣?
}



void TIM4_PWM_Init(u32 arr,u32 psc)
{		 					 
	//麓脣虏驴路脰脨猫脢脰露炉脨脼赂脛IO驴脷脡猫脰脙
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  	//TIM14脢卤脰脫脢鹿脛脺    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	//脢鹿脛脺PORTF脢卤脰脫	
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15,GPIO_AF_TIM4); //GPIOF9赂麓脫脙脦陋露篓脢卤脝梅14
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4); //GPIOF9赂麓脫脙脦陋露篓脢卤脝梅14
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15 | GPIO_Pin_14 | GPIO_Pin_13 | GPIO_Pin_12;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //赂麓脫脙鹿娄脛脺
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//脣脵露脠100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //脥脝脥矛赂麓脫脙脢盲鲁枚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //脡脧脌颅
	GPIO_Init(GPIOD,&GPIO_InitStructure);              //鲁玫脢录禄炉PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //露篓脢卤脝梅路脰脝碌
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //脧貌脡脧录脝脢媒脛拢脢陆
	TIM_TimeBaseStructure.TIM_Period=arr;   //脳脭露炉脰脴脳掳脭脴脰碌
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//鲁玫脢录禄炉露篓脢卤脝梅14
	
	//鲁玫脢录禄炉TIM14 Channel1 PWM脛拢脢陆	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //脩隆脭帽露篓脢卤脝梅脛拢脢陆:TIM脗枚鲁氓驴铆露脠碌梅脰脝脛拢脢陆2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //卤脠陆脧脢盲鲁枚脢鹿脛脺
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //脢盲鲁枚录芦脨脭:TIM脢盲鲁枚卤脠陆脧录芦脨脭碌脥
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  //赂霉戮脻T脰赂露篓碌脛虏脦脢媒鲁玫脢录禄炉脥芒脡猫TIM1 4OC1
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  //脢鹿脛脺TIM14脭脷CCR1脡脧碌脛脭陇脳掳脭脴录脛麓忙脝梅
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE脢鹿脛脺 
	
	TIM_Cmd(TIM4, ENABLE);  //脢鹿脛脺TIM14									  
}  

void TIM5_PWM_Init(u32 arr,u32 psc)
{		 					 
	//麓脣虏驴路脰脨猫脢脰露炉脨脼赂脛IO驴脷脡猫脰脙
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM14脢卤脰脫脢鹿脛脺    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//脢鹿脛脺PORTF脢卤脰脫	
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); //GPIOF9赂麓脫脙脦陋露篓脢卤脝梅14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //赂麓脫脙鹿娄脛脺
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//脣脵露脠100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //脥脝脥矛赂麓脫脙脢盲鲁枚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //脡脧脌颅
	GPIO_Init(GPIOA,&GPIO_InitStructure);              //鲁玫脢录禄炉PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //露篓脢卤脝梅路脰脝碌
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //脧貌脡脧录脝脢媒脛拢脢陆
	TIM_TimeBaseStructure.TIM_Period=arr;   //脳脭露炉脰脴脳掳脭脴脰碌
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//鲁玫脢录禄炉露篓脢卤脝梅14
	
	//鲁玫脢录禄炉TIM14 Channel1 PWM脛拢脢陆	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //脩隆脭帽露篓脢卤脝梅脛拢脢陆:TIM脗枚鲁氓驴铆露脠碌梅脰脝脛拢脢陆2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //卤脠陆脧脢盲鲁枚脢鹿脛脺
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //脢盲鲁枚录芦脨脭:TIM脢盲鲁枚卤脠陆脧录芦脨脭碌脥
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE脢鹿脛脺 
	
	TIM_Cmd(TIM5, ENABLE);  //脢鹿脛脺TIM14									  
}  


void TIM1_PWM_Init(u32 arr,u32 psc)
{	
    GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  //TIM4 鏃堕挓浣胯兘
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); //浣胯兘 PORTB 鏃堕挓

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1); //PB7 澶嶇敤涓?TIM4

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;		   //GPIOB5
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;	   //澶嶇敤鍔熻兘
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //閫熷害 50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	 //鎺ㄦ尳澶嶇敤杈撳嚭
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;	   //涓婃媺
	GPIO_Init(GPIOA, &GPIO_InitStructure);			   //鍒濆鍖?PB7

	TIM_TimeBaseStructure.TIM_Prescaler = psc;					//瀹氭椂鍣ㄥ垎棰?
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; //鍚戜笂璁℃暟妯″紡
	TIM_TimeBaseStructure.TIM_Period = arr;						//鑷姩閲嶈杞藉€?
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //鍒濆鍖栧畾鏃跺櫒 4

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //PWM 璋冨埗妯″紡 2

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //姣旇緝杈撳嚭浣胯兘
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;	 //杈撳嚭鏋佹€т綆
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);					  //鍒濆鍖栧璁?TIM1 4OC1
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);			  //浣胯兘棰勮杞藉瘎瀛樺櫒
	TIM_ARRPreloadConfig(TIM1, ENABLE);							  //ARPE 浣胯兘
	TIM_CtrlPWMOutputs(TIM1, ENABLE);	
	TIM_Cmd(TIM1, ENABLE);										  //浣胯兘 TIM4
	
}  

void TIM8_PWM_Init(u32 arr,u32 psc)
{		 					 
	//麓脣虏驴路脰脨猫脢脰露炉脨脼赂脛IO驴脷脡猫脰脙
	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);  	//TIM14脢卤脰脫脢鹿脛脺    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 	//脢鹿脛脺PORTF脢卤脰脫	
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8); //GPIOF9赂麓脫脙脦陋露篓脢卤脝梅14
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;           //GPIOF9
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //赂麓脫脙鹿娄脛脺
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//脣脵露脠100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //脥脝脥矛赂麓脫脙脢盲鲁枚
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //脡脧脌颅
	GPIO_Init(GPIOC,&GPIO_InitStructure);              //鲁玫脢录禄炉PF9
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //露篓脢卤脝梅路脰脝碌
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //脧貌脡脧录脝脢媒脛拢脢陆
	TIM_TimeBaseStructure.TIM_Period=arr;   //脳脭露炉脰脴脳掳脭脴脰碌
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//鲁玫脢录禄炉露篓脢卤脝梅14
	
	//鲁玫脢录禄炉TIM14 Channel1 PWM脛拢脢陆	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //脩隆脭帽露篓脢卤脝梅脛拢脢陆:TIM脗枚鲁氓驴铆露脠碌梅脰脝脛拢脢陆2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //卤脠陆脧脢盲鲁枚脢鹿脛脺
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //脢盲鲁枚录芦脨脭:TIM脢盲鲁枚卤脠陆脧录芦脨脭碌脥
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);

	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
	
//	TIM_SelectCOM(TIM1,ENABLE);
	
	
  TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPE脢鹿脛脺 
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	
	TIM_Cmd(TIM8, ENABLE);  //脢鹿脛脺TIM14									  
}  

/*********************************WIFI*************************/
/**************************************************************/
