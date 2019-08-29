#include "light.h"

#include "can.h"
#include "moveBase.h"

/*红场亮红灯，蓝场亮蓝灯，复位好了亮青色灯，大腿出问题亮绿灯，小腿出问题亮金色灯，全出问题亮白灯*/

/** 
  * @brief  亮白灯
  * @note
  * @param  None
  * @retval None
  */
  
void White(uint8_t ElmoNum)
{
	 uint32_t data[1][2]={
							0x5247424C,0xFFFFFF00,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  亮红灯
  * @note
  * @param  None
  * @retval None
  */

void Red(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFF000000,   
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}
	
/** 
  * @brief  亮绿灯
  * @note
  * @param  None
  * @retval None
  */

void Green(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x00FF0000,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  亮蓝灯
  * @note
  * @param  None
  * @retval None
  */
void Blue(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x0000FF00,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  亮金灯
  * @note
  * @param  None
  * @retval None
  */
void Gold(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFFFF0000,    
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  亮紫灯
  * @note
  * @param  None
  * @retval None
  */
void Purple(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFF00FF00,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}


/** 
  * @brief  亮青灯
  * @note
  * @param  None
  * @retval None
  */
void Blue_green(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x00FFFF00,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  亮粉色灯
  * @note
  * @param  None
  * @retval None
  */
void Pink(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFF003200,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  亮黄绿灯
  * @note
  * @param  None
  * @retval None
  */
void Kelly(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0xFFFF0000,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard ;			 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data  ;			 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}

/** 
  * @brief  灭灯
  * @note
  * @param  None
  * @retval None
  */
void LightsOff(uint8_t ElmoNum)
{
	uint32_t data[1][2]={
							0x5247424C,0x00000000,     
						 };
	uint8_t mbox;
	CanTxMsg TxMessage;
						 
	TxMessage.StdId=ELMO_DEVICE_BASEID+ElmoNum;					// standard identifier=0
	TxMessage.ExtId=ELMO_DEVICE_BASEID+ElmoNum;					// extended identifier=StdId
	TxMessage.IDE=CAN_Id_Standard;		 						// type of identifier for the message is Standard
	TxMessage.RTR=CAN_RTR_Data;		 						// the type of frame for the message that will be transmitted
	TxMessage.DLC=8;

   	TxMessage.Data[0] = *(unsigned long*)&data[0][0]&0xff;
	TxMessage.Data[1] = (*(unsigned long*)&data[0][0]>>8)&0xff;
	TxMessage.Data[2] = (*(unsigned long*)&data[0][0]>>16)&0xff;
	TxMessage.Data[3] = (*(unsigned long*)&data[0][0]>>24)&0xff;
	TxMessage.Data[4] =  *(unsigned long*)&data[0][1]&0xff;
	TxMessage.Data[5] = (*(unsigned long*)&data[0][1]>>8)&0xff;
	TxMessage.Data[6] = (*(unsigned long*)&data[0][1]>>16)&0xff;
	TxMessage.Data[7] = (*(unsigned long*)&data[0][1]>>24)&0xff;

	mbox= CAN_Transmit(CAN2, &TxMessage);

	uint16_t timeout = 0;
	while((CAN_TransmitStatus(CAN2, mbox)!= CAN_TxStatus_Ok))
	{
		timeout++;
		if(timeout > 3000)
		{
			USART_OUT(UART4,(uint8_t*)"can send err\r\n");
			break;
		}
		timeout = 0;
	} 
}



/** 
* @brief  陀螺初始化完亮白灯
  * @note
  * @param  None
  * @retval None
  */
void WhiteSite(void)
{
	White(LEGONE_LIGHT_ID);
	White(LEGTWO_LIGHT_ID);
	White(LEGTHR_LIGHT_ID);
	White(LEGFOR_LIGHT_ID);

}

/** 
* @brief  红场亮红灯
  * @note
  * @param  None
  * @retval None
  */
void RedSite(void)
{
	Red(LEGONE_LIGHT_ID);
	Red(LEGTWO_LIGHT_ID);
	Red(LEGTHR_LIGHT_ID);
	Red(LEGFOR_LIGHT_ID);
}

/** 
* @brief  红场亮红灯
  * @note
  * @param  None
  * @retval None
  */
void BlueSite(void)
{
	Blue(LEGONE_LIGHT_ID);
	Blue(LEGTWO_LIGHT_ID);
	Blue(LEGTHR_LIGHT_ID);
	Blue(LEGFOR_LIGHT_ID);
}

/** 
* @brief 	机器人姿态初始化完成亮青色灯
  * @note
  * @param  None
  * @retval None
  */
void BrightBlue_green(void)
{
	Blue_green(LEGONE_LIGHT_ID);
	Blue_green(LEGTWO_LIGHT_ID);
	Blue_green(LEGTHR_LIGHT_ID);
	Blue_green(LEGFOR_LIGHT_ID);
}



/** 
  * @brief 	定位系统初始化完成亮粉色灯
  * @note
  * @param  None
  * @retval None
  */
void BrightPink(void)
{
	Pink(LEGONE_LIGHT_ID);
	Pink(LEGTWO_LIGHT_ID);
	Pink(LEGTHR_LIGHT_ID);
	Pink(LEGFOR_LIGHT_ID);

}


void AllGreen(void)
{
	Green(LEGONE_LIGHT_ID);
	Green(LEGTWO_LIGHT_ID);
	Green(LEGTHR_LIGHT_ID);
	Green(LEGFOR_LIGHT_ID);

}

/** 
  * @brief 	云台初始化完成亮粉色灯
  * @note
  * @param  None
  * @retval None
  */
void BrightPurple(void)
{
	Purple(LEGONE_LIGHT_ID);
	Purple(LEGTWO_LIGHT_ID);
	Purple(LEGTHR_LIGHT_ID);
	Purple(LEGFOR_LIGHT_ID);
	
}

	



