#include "sensor.h"
#include "vloop.h"
#include "module_RED.h"
#include "moveBase.h"
#include "cloudplatform.h"
#include "app_cfg.h"
#include "errorHand.h"
#include "balance.h"
#include "dma.h"
/***************全局变量*********************/

coordinateGet_ coordinateGet;

/*视觉信息结构体*/
visual_ visual;

/*判断视觉通讯是都正常变量*/
static uint8_t checkFlag=0;

/*接收到视觉信息标志位*/
uint8_t receiveFlag=0;

uint8_t ropeAdjustFlag=0;
/*视觉返回正确的值*/
uint8_t dataRight=0;

union trans_t
{
	float data_32;
	uint8_t data_8[4];
}trans;

/***************extern*********************/
extern leg_ legOne, legTwo, legThr, legFor;
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern uint8_t site;
extern uint16_t timeRecord;
extern int can1ErrNum[8];
extern int can2ErrNum[4];
extern uint8_t dataRight;

void USART2_IRQHandler(void)
{
		static uint8_t ch;
		static uint8_t count=0;
		static uint8_t i=0;
		OS_CPU_SR  cpu_sr;
		OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
		OSIntNesting++;
		OS_EXIT_CRITICAL();

		if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)   
		{
			USART_ClearITPendingBit(USART2,USART_IT_RXNE);
			ch=USART_ReceiveData(USART2);
//			USART_SendData(UART4,ch);	
			switch(count)
			{
				case 0:
					if(ch=='\r'||ch=='o')
						count++;
					else if(ch=='g')
					{
						count=6;
					}
					else
						count=0;
				break;

				case 1:
					if(ch=='\n')
					{
						i=0;
						count++;
					}
					else if(ch=='k')
					{
						count=4;
					}
					else
						count=0;
				break;
				
				case 2:
					switch(ch)
					{
						/*
							status为当前视觉信息标识变量，其中：
							0：只有机器人到侧墙的距离；
							1：加入机器人到台阶的距离，但是在接近台阶的时候只有到台阶的距离，没有到侧墙的距离；
							2：机器人到前面墙的距离，以及到侧墙的距离；
							3：机器人到第二条绳的距离，以及到侧墙的距离；
							4：机器人到前面墙的距离，以及到坡下的距离；
							5：机器人到前面墙的距离，没有到侧墙的距离：
							6：机器人到前面墙的距离，以及到坡上的距离；
							7：机器人到前面墙的距离，以及到中间墙的距离；
						*/
						case '0':
							visual.status=0;
							break;
						case '1':
							visual.status=1;
							break;
						case '2':
							visual.status=2;
							break;
						case '3':
							visual.status=3;
							break;
						case '4':
							visual.status=4;
							break;
						case '5':
							visual.status=5;
							break;
						case '6':
							visual.status=6;
							break;
						case '7':
							visual.status=7;
							break;
						default:break;
					}
					
					count++;
				break;
					
				case 3:
					coordinateGet.data[i]=ch;
					i++;
					if(i>=16)
					{
						i=0;
						count++;
					}
						
				break;

				case 4:
					if(ch=='\r')
						count++;
					else
						count=0;
				break;

				case 5:
					if(ch=='\n')
					{
						checkFlag=1;
						
						
						/*视觉距离信息，第一个为摄像头前向物体到摄像头距离，不管在哪个阶段，其总为摄像头前方物体距离*/
						visual.distance_C[0]=coordinateGet.value[0];
						
						/*第二个为为摄像头侧向物体到摄像头距离*/
						visual.distance_C[1]=coordinateGet.value[1];
						
						visual.angle=coordinateGet.value[2];
						
						/*视觉图像处理时间*/
						visual.time=coordinateGet.value[3];
						
						/*此时视觉处理的图片的偏航角为上一次记录的偏航角*/
						visual.recordYawLast=visual.recordYaw;
						
						/*考虑到摄像头处理完就会取下一张图片，即认为此次的偏航角即为下一时刻处理的图片的偏航角*/
						visual.recordYaw=quadrupedStatus.euler[2];
						
						if(fabsf(visual.distance_C[1] - 675.0f) < 50.0f)
							dataRight=1;
						
						if(JudgeInformationBeyondVenue() == 1)
						{
                            ropeAdjustFlag=1;
							receiveFlag=1;
						}
						else;
						
					}
					count=0;
				break;
				case 6:
					if(ch == 'o')
					{
						count++;
					}
				break;
				case 7:
					if(ch == '\r')
					{
						count++;
					}
					else
						count=0;
				break;
				case 8:
					if(ch == '\n')
					{
						visual.goFlag=1;
						count=0;
					}
					else
						count=0;
				break;			
				default:
					count=0;
				break;		 
			}
		}
		else
		{
			USART_ClearITPendingBit(USART2, USART_IT_PE);
			USART_ClearITPendingBit(USART2, USART_IT_TXE);
			USART_ClearITPendingBit(USART2, USART_IT_TC);
			USART_ClearITPendingBit(USART2, USART_IT_ORE_RX);
			USART_ClearITPendingBit(USART2, USART_IT_IDLE);
			USART_ClearITPendingBit(USART2, USART_IT_LBD);
			USART_ClearITPendingBit(USART2, USART_IT_CTS);
			USART_ClearITPendingBit(USART2, USART_IT_ERR);
			USART_ClearITPendingBit(USART2, USART_IT_ORE_ER);
			USART_ClearITPendingBit(USART2, USART_IT_NE);
			USART_ClearITPendingBit(USART2, USART_IT_FE);
			USART_ReceiveData(USART2);
		}
		
		OSIntExit();
}

/**
  * @brief  检查通讯是否正常
  * @note
  * @param  None
  * @retval None
  */
void CheckVisionCommunication(uint8_t gaitCnt_t,float yaw,uint16_t timeRecord_t)
{
	uint8_t i = 0;
	uint8_t tdata[12];
	static uint8_t visionCnt=0;
	trans.data_32=yaw;
	
	tdata[0]='a';
	tdata[1]='t';
	tdata[2]='+';
	if(site == RED)
		tdata[3]='0';
	else if (site == BLUE)
		tdata[3]='1';
	tdata[4]='+';
	
	switch(gaitCnt_t)
	{
		case 9:
			tdata[5]='3';
			break;
		case 15:
			tdata[5]='6';
			break;
		case 29:
			tdata[5]='9';
			break;
		default:
			tdata[5]='0';
			break;
	}
	tdata[6]=trans.data_8[0];
	tdata[7]=trans.data_8[1];
	tdata[8]=trans.data_8[2];
	tdata[9]=trans.data_8[3];
	
	tdata[10]='\r';
	tdata[11]='\n';
	visionCnt++;
	
	if(visionCnt >= (int)(300/timeRecord_t))
	{
		visionCnt=0;
		for(i=0;i<12;i++)
		USART_SendData(USART2,tdata[i]);
	}		
}


/**
  * @brief  发送测试阶段命令
  * @note
  * @param  None
  * @retval None
  */
void SendTestCmd(float yaw)
{
    uint8_t sendCnt=0;
	uint8_t i = 0;
	uint8_t tdata[12]={0};
	trans.data_32=yaw;
	
	tdata[0]='a';
	tdata[1]='t';
	tdata[2]='+';
	if(site == RED)
		tdata[3]='0';
	else if (site == BLUE)
		tdata[3]='1';
	else
		tdata[3]='0';
	
	tdata[4]='+';
	tdata[5]='0';
	tdata[6]=trans.data_8[0];
	tdata[7]=trans.data_8[1];
	tdata[8]=trans.data_8[2];
	tdata[9]=trans.data_8[3];
	
	tdata[10]='\r';
	tdata[11]='\n';

	while(!checkFlag&&(sendCnt>150))
	{
		delay_ms(1);
		for(i=0;i<12;i++)
             USART_SendData(USART2,tdata[i]);	
         sendCnt++;
	}
	USART_OUT(UART4,(uint8_t *)" vision  communication is  ok !\r\n");
	
//	while(!dataRight)
//	{
//		delay_ms(10);
//		CheckVisionCommunication(0,quadrupedStatus.euler[2],100);
//		USART_OUT(UART4,(uint8_t *)"%d %d %d\r\n",(int)(visual.status),(int)(visual.distance_C[0]),(int)(visual.distance_C[1]));
//	}
//	
//	USART_OUT(UART4,(uint8_t *)" vision  data is  ok !\r\n");
}
/** 
  * @brief  给小电脑发送已停止命令
  * @note
  * @param  None
  * @retval None
  */
void SendStopReady(void)
{
	uint8_t i = 0;
	uint8_t tdata[8];

	tdata[0]='a';
	tdata[1]='t';
	tdata[2]='+';
	if(site == RED)
		tdata[3]='0';
	else if (site == BLUE)
		tdata[3]='1';
	tdata[4]='+';
	tdata[5]='s';
	tdata[6]='\r'; 
	tdata[7]='\n';

	for(i=0;i<8;i++)
	 USART_SendData(USART2,tdata[i]);	
}


/**
  * @brief  发送读电机位置的命令
  * @note
  * @param  None
  * @retval None
  */
extern int testTime;
extern uint8_t testFlag;
void GetJointPos(void)
{
	extern uint16_t gaitCnt;
	
	testFlag=1;
	/*按广播号读肩关节和膝关节电机位置*/
	ReadActualPos(CAN1,ELMO_BROADCAST_ID);
	
	/*按ID号读肩关节和膝关节电机位置*/
//	ReadActualPos(CAN1,1);
	can1ErrNum[0]++;
//	ReadActualPos(CAN1,2);
	can1ErrNum[1]++;
	
//	ReadActualPos(CAN1,3);
	can1ErrNum[2]++;
//	ReadActualPos(CAN1,4);
	can1ErrNum[3]++;
	
//	ReadActualPos(CAN1,5);
	can1ErrNum[4]++;
//	ReadActualPos(CAN1,6);
	can1ErrNum[5]++;
	
//	ReadActualPos(CAN1,7);
	can1ErrNum[6]++;
//	ReadActualPos(CAN1,8);
	can1ErrNum[7]++;
	
	ReadActualPos(CAN2,ELMO_BROADCAST_ID);
	
	/*按ID号读航向电机位置*/
//	ReadActualPos(CAN2,legOne.ID);
	can2ErrNum[0]++;
//	ReadActualPos(CAN2,legTwo.ID);
	can2ErrNum[1]++;
//	ReadActualPos(CAN2,legThr.ID);
	can2ErrNum[2]++;
//	ReadActualPos(CAN2,legFor.ID);
	can2ErrNum[3]++;
	if(gaitCnt>=46)
		ReadActualPos(CAN2,LIFT_UP_ID);
		
}

/**
  * @brief  发送读电机速度的命令
  * @note
  * @param  None
  * @retval None
  */
void GetJointSpeed(void)
{
	ReadActualVel(CAN1,ELMO_BROADCAST_ID);
}	


/**
  * @brief  读3508的速度和位置
  * @note   7号电机控制x轴，左前右后
  * @note   6号电机控制y轴，右前左后
  * @param  None
  * @retval None
       
  */

void GetGyroMessage(void)
{
	extern BalancerType gBalancer1;
	extern BalancerType gBalancer2;
	ReadActualPos(CAN2 ,ELMO_BROADCAST_ID_2);

	gBalancer2.rollAngle = DEGREE_TO_RAD(gBalancer2.sysPos.angle[0]);
	gBalancer2.rollAngleSpeed = DEGREE_TO_RAD(gBalancer2.sysPos.angularVelocity[0]);

	gBalancer1.rollAngle = DEGREE_TO_RAD(gBalancer1.sysPos.angle[1]);
	gBalancer1.rollAngleSpeed = DEGREE_TO_RAD(gBalancer1.sysPos.angularVelocity[1]);
        
}


/**
  * @brief  判断视觉信息是否超出场地
  * @note 
  * @param  None
  * @retval None
  */
uint8_t JudgeInformationBeyondVenue(void)
{
	float judgeDistance[2][8]={{2300.0f,2000.0f,2000.0f,2380.0f,2420.0f,1450.0f,1450.0f,1450.0f},\
							   {1900.0f,2900.0f,2892.0f,2892.0f,1500.0f,0.0000f,1575.0f,1620.0f}};   
	if(visual.status == 0)
	{
		if(visual.distance_C[0] < judgeDistance[0][0] && visual.distance_C[0] > 1.0f && visual.distance_C[1] < judgeDistance[1][0] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0;
	}
	else if(visual.status == 1)
	{
		if(visual.distance_C[0] < judgeDistance[0][1] && visual.distance_C[1] < judgeDistance[1][1] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0; 
	}
	else if(visual.status == 2)
	{
		if(visual.distance_C[0] < judgeDistance[0][2] && visual.distance_C[0] > 1.0f && visual.distance_C[1] < judgeDistance[1][3] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0;
	}	
	else if(visual.status == 3)
	{
		if(visual.distance_C[0] < judgeDistance[0][3] && visual.distance_C[1] < judgeDistance[1][3] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0;
	}	
	else if(visual.status == 4)
	{
		if(visual.distance_C[0] < judgeDistance[0][4] && visual.distance_C[0] > 1.0f && visual.distance_C[1] < judgeDistance[1][4] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0;
	}	
	else if(visual.status == 5)
	{
		if(visual.distance_C[0] < judgeDistance[0][5] && visual.distance_C[0] > 1.0f)
			return 1;
		else 
			return 0;
	}	
	else if(visual.status == 6)
	{
		if(visual.distance_C[0] < judgeDistance[0][6] && visual.distance_C[0] > 1.0f && visual.distance_C[1] < judgeDistance[1][6] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0;
	}	
	else if(visual.status == 7)
	{
		if(visual.distance_C[0] < judgeDistance[0][7] && visual.distance_C[0] > 1.0f && visual.distance_C[1] < judgeDistance[1][7] && visual.distance_C[1] > 1.0f)
			return 1;
		else 
			return 0;
	}
	else
		return 0;
}


/**
  * @brief  判断视觉信息是否与预测差很远
  * @note 
  * @param  None
  * @retval None
  */
void JudgeInformation(float compensateX,float compensateY)
{
	
	float angle=0,length=0;
	float estimatedPos[2]={0};
	
	estimatedPos[0]=quadrupedStatus.estimatedPos[0];
	estimatedPos[1]=quadrupedStatus.estimatedPos[1];
	angle=atan2f(compensateY,compensateX);
	length=sqrt(Square(compensateY)+Square(compensateX));
	
	if(site == RED)
	{
		if(visual.status == 0)
		{
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+length*cosf(angle); 
		}
			
		else if(visual.status == 1)
		{
			if(visual.statusLast == 0)
			{
				quadrupedStatus.estimatedPos[0]=(estimatedPos[0]+estimatedPos[1]+191.4214f)/sqrtf(2.0f);
				quadrupedStatus.estimatedPos[1]=(estimatedPos[0]-estimatedPos[1]+2400.0f)/sqrtf(2.0f);
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(-45.0f*PI/180.0f+angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(-45.0f*PI/180.0f+angle);
		}

		else if(visual.status == 2)
		{
			if(visual.statusLast == 1)
			{
				quadrupedStatus.estimatedPos[0]=(estimatedPos[0]+estimatedPos[1]+996.0155f)/sqrtf(2.0f);
				quadrupedStatus.estimatedPos[1]=(estimatedPos[0]-estimatedPos[1]+4390.128f)/sqrtf(2.0f);
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+length*cosf(angle); 
		}
		else if(visual.status == 3)
		{
			if(visual.statusLast == 2)
			{
				quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]+880;
				
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+length*cosf(angle); 
		}
		else if(visual.status == 4)
		{
			if(visual.statusLast == 3)
			{
				quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]+1620.0f;
				quadrupedStatus.estimatedPos[1]=1450.0f-quadrupedStatus.estimatedPos[1];
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(angle); 
		}
		else if(visual.status == 5)
		{
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=0.0f; 
		}
		else if(visual.status == 6)
		{
			if(visual.statusLast == 5)
			{
				quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+1575.0f;
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(angle); 
		}
		else if(visual.status == 7)
		{
			if(visual.statusLast ==6)
			{
				quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+1100.0f;
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(angle); 
		}
		
	}
	/*蓝场*/
	else
	{
		if(visual.status == 0)
		{
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(angle); 
		}
			
		else if(visual.status == 1)
		{
			if(visual.statusLast == 0)
			{
				quadrupedStatus.estimatedPos[0]=(estimatedPos[0]+estimatedPos[1]+191.4214f)/sqrtf(2.0f);
				quadrupedStatus.estimatedPos[1]=(estimatedPos[0]-estimatedPos[1]+2400.0f)/sqrtf(2.0f);
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*cosf(-45.0f*PI/180.0f+angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*sinf(-45.0f*PI/180.0f+angle);
		}
		
		else if(visual.status == 2)
		{
			if(visual.statusLast == 1)
			{
				quadrupedStatus.estimatedPos[0]=(estimatedPos[0]+estimatedPos[1]+996.0155f)/sqrtf(2.0f);
				quadrupedStatus.estimatedPos[1]=(estimatedPos[0]-estimatedPos[1]+4390.128f)/sqrtf(2.0f);
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(angle); 
		}
		else if(visual.status == 3)
		{
			if(visual.statusLast == 2)
			{
				quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]+880;
				
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]-length*cosf(angle);
		}
		else if(visual.status == 4)
		{
			if(visual.statusLast == 3)
			{
				quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]+1620.0f;
				quadrupedStatus.estimatedPos[1]=1450.0f-quadrupedStatus.estimatedPos[1];
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+length*cosf(angle); 
		}                                                              
		else if(visual.status == 5)                                    
		{                                                              
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=0.0f;                      
		}                                                              
		else if(visual.status == 6)                                    
		{    
			if(visual.statusLast == 5)
			{
				quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+1575.0f;
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+length*cosf(angle);
		}                                                              
		else if(visual.status == 7)                                     
		{
			if(visual.statusLast ==6)
			{
				quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+1100.0f;
			}
			quadrupedStatus.estimatedPos[0]=quadrupedStatus.estimatedPos[0]-length*sinf(angle);
			quadrupedStatus.estimatedPos[1]=quadrupedStatus.estimatedPos[1]+length*cosf(angle);
		}
	}
	
}
	











