 #include "balance.h"
#include "stdint.h"
#include "elmo.h"
#include "usart.h"
#include "timer.h"
#include "elmo.h"
#include "math.h"
#include "moveBase.h"
#include "pid.h"
#include "module_RED.h"
#include "vloop.h"
BalancerType gBalancer1;
BalancerType gBalancer2;
float M3508Speed[2]={0};

float controlSpeed = 0;
float controlSpeedRecord = 0;
float imuSpeed = 0;
float encoderSpeed = 0;
char forwardFlag=0;
extern leg_ legOne, legTwo, legThr, legFor;
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern float desireAngle_X,desireAngle_Y;

/***
 @author Oliver
 @brief 
 @para  输入陀螺的角速度rad/s
@note  为了方便计算，直接输入陀螺角速度  
***/
void balanceCtrlInit()
{    
    gBalancer1.runStatus =RESET_STATUS;
	gBalancer2.runStatus =RESET_STATUS;
	gBalancer1.gyroZeroPos = DEGREE_TO_RAD(ZERO_GYRO_POS);
	gBalancer2.gyroZeroPos = DEGREE_TO_RAD(ZERO_GYRO_POS);
    gBalancer1.cnt=0;
    gBalancer2.cnt=0;
}

/***
 @author Oliver
 @brief 陀螺控制 
 @para  输入陀螺的角速度rad/s
@note  为了方便计算，直接输入电机角速度  
***/
void BalancerMotorControl(float w,uint8_t ID)//
{
//	USART_OUT(USART2,"%d\t",(int)( M3508_RATIO * GEAR_RATIO * RAD_TO_PULSE(w)));
    if(ID==1)
	VelCrl2(CAN2,6,(int)(RAD_TO_PULSE(w)));
    if(ID==2)
    VelCrl2(CAN2,7,(int)(RAD_TO_PULSE(w)));
    
}


/***
 @author Oliver
 @brief  回归 
 @para  
 @note  基本思想和斩波类似，为了不受限制于限位
 @ret  返回3508角速度
***/
#define NEGATIVE_ADJUST 1
#define POSITIVE_ADJUST 2

#define START_ADJUST 10
#define RESET_ADJUST 20

/***
 @author Oliver
 @brief 陀螺控制 
 @para  输入陀螺的角速度rad/s
 //K[3]: 摆的角度，摆的速度，3508角度
 @note  为了方便计算，直接输入角速度  
***/
//static float gainK1[3] = {3500.2f,700.f,-100.f};
//static float gainK2[3] = {3500.2f,700.f,-100.f};

//static float zs[3] = {DEGREE_TO_RAD((ZERO_GYRO_POS)),0.f,ZERO_ENCODER_POS};
float u1 = 0;
float u2 = 0;
/*
只有知道运动趋势的情况下，才使用这种前馈的陀螺控制方式。
  F
   ^
   |period
   |-----
   |   /|   /|   /|   /
  0|  / |  / |  / |  /  
   |-/--| /--|-/--|-/------
   |/   |/   |/   |/   
   |
   |
   |-------------------------->

*/
/***
 @author Oliver
 @brief 根据步态进行陀螺的控制
 @para  period 为摆动相周期

 @note  根据陀螺建模后开环给出陀螺速度
***/

uint8_t closeLoopFlag=0;
float u1Rrcord=0,u2Rrcord=0;
float closeLoopRecord1=0,closeLoopRecord2=0;
float closeLoopForward_y = 0,closeLoopForward_x  = 0;
extern uint16_t gaitCnt;

extern float gyro1InitPos;
extern float gyro2InitPos;

#define SIGN(vel) (vel > 0 ? 1 : (vel < 0 ? -1 : 0))
void BalanceOpenLoopControl_y(void)
{
	float closeLoop=0;

	if(closeLoopFlag == 1)
		closeLoop=RF_LBAngleControl(desireAngle_Y,quadrupedStatus.euler[1],quadrupedStatus.gyro[1]) + closeLoopForward_y;
	else
		closeLoop=0;
	
	if(gBalancer1.motor.posPulse >=GYRO1_PROTECT_MAX)
	{
		if(closeLoop > 0)
			closeLoop=0;
	}
	else if(gBalancer1.motor.posPulse <= GYRO1_PROTECT_MIN)
	{
		if(closeLoop < 0)
			closeLoop=0;
	}

	closeLoopRecord1=closeLoop;
	
	M3508Speed[0]=closeLoop;
	
}

/***
 @author Oliver
 @brief 陀螺行程计算 
 @para  none
 @note  计算在规定时间内陀螺所能走的大致行程。默认陀螺控制周期为5ms。0.5*0.5*period = dis
***/
float CalculBalancer1Dis(float openLoopPeriod, float openLoopVel)
{

	float openLoopDis = 0;
	openLoopDis = RAD_TO_DEGREE((0.5f * openLoopVel / M3508_RATIO / GEAR_RATIO * openLoopPeriod * 0.5f * 0.005f));
	return openLoopDis;
}

/***
 @author Oliver
 @brief 陀螺复位 
 @para  none
 @note  统一以头部的两个陀螺作为参照，因此不需要考虑正负 
***/
uint8_t gyro1InitFlag = 0;
int GoToOpenLoopInitPos1(float zeroMotorPos1, float openLoopDis)
{
	float vel = 0.f;
	if(gyro1InitPos<zeroMotorPos1)
	{
		vel=50;
		if(gBalancer1.motor.pos < zeroMotorPos1 )
		{
			BalancerMotorControl(vel,1);
		}
		else
		{
			BalancerMotorControl(0,1);
			gyro1InitFlag = 1;
		}
		
		if(gBalancer1.motor.posPulse >=GYRO1_PROTECT_MAX )
		{
			vel = 0;
			BalancerMotorControl(vel,1);
		}
	}
	else 
	{
		vel=-50;
		if(gBalancer1.motor.pos > zeroMotorPos1 )
		{
			BalancerMotorControl(vel,1);
		}
		else
		{
			BalancerMotorControl(0,1);
			gyro1InitFlag = 1;
		}
		
		if(gBalancer1.motor.posPulse <=GYRO1_PROTECT_MIN )
		{
			vel = 0;
			BalancerMotorControl(vel,1);
		}
	}
	return gyro1InitFlag;
}



/***
 @author Oliver
 @brief 根据步态进行陀螺的控制
 @para  period 为摆动相周期

 @note  根据陀螺建模后开环给出陀螺速度
***/
#define SIGN(vel) (vel > 0 ? 1 : (vel < 0 ? -1 : 0))
void BalanceOpenLoopControl_x(void)
{	

	float closeLoop=0;

	if(closeLoopFlag == 1)
		closeLoop=LF_RBAngleControl(desireAngle_X,quadrupedStatus.euler[0],quadrupedStatus.gyro[0]) + closeLoopForward_x;
	else
		closeLoop=0;
	
	if(gBalancer2.motor.posPulse >=GYRO2_PROTECT_MAX)
	{

		if(closeLoop > 0)
			closeLoop=0;
	}
	else if(gBalancer2.motor.posPulse <= GYRO2_PROTECT_MIN)
	{

		if(closeLoop < 0)
			closeLoop=0;
	}		

	closeLoopRecord2=closeLoop;
	M3508Speed[1]=closeLoop;
}

/***
 @author Oliver
 @brief 陀螺行程计算 
@para  参数均为正
 @note  计算在规定时间内陀螺所能走的大致行程。默认陀螺控制周期为5ms
***/
float CalculBalancer2Dis(float openLoopPeriod, float openLoopVel)
{

	float openLoopDis = 0;
	openLoopDis = RAD_TO_DEGREE((0.5f * openLoopVel / M3508_RATIO / GEAR_RATIO * openLoopPeriod * 0.5f * 0.005f));
	return openLoopDis;
}

/***
 @author Oliver
 @brief 陀螺复位 
 @para  none
 @note  统一以头部的两个陀螺作为参照，因此不需要考虑正负 
***/
uint8_t gyro2InitFlag = 0;
int GoToOpenLoopInitPos2(float zeroMotorPos2, float openLoopDis)
{
	float vel = 0.f;
	if(gyro2InitPos<zeroMotorPos2 )
	{
		vel=50;
		if(gBalancer2.motor.pos < zeroMotorPos2 )
		{
			BalancerMotorControl(vel,2);
		}
		else
		{
			BalancerMotorControl(0,2);
			gyro2InitFlag = 1;
		}
		
		if(gBalancer2.motor.posPulse >= GYRO2_PROTECT_MAX )
		{
			vel = 0;
			BalancerMotorControl(vel,2);
		}
	}
	else 
	{
		vel=-50;
		if(gBalancer2.motor.pos > zeroMotorPos2 )
		{
			BalancerMotorControl(vel,2);
		}
		else
		{
			BalancerMotorControl(0,2);
			gyro2InitFlag = 1;
		}
		
		if(gBalancer2.motor.posPulse <= GYRO2_PROTECT_MIN )
		{
			vel = 0;
			BalancerMotorControl(vel,2);
		}		
	}
//	USART_OUT(UART5,(uint8_t*)"%d\t%d\t",flag,(int)(zeroMotorPos2  + 0.5f * openLoopDis));
	
	return gyro2InitFlag;
}

/** 
  * @brief  陀螺复位
  * @note
  * @param  None
  * @retval None 
  */
void GyroBack_y(float backPos,uint8_t *backFlag)
{
	float vel = 0.f;
	static int direct = 0,status=0;
	static float backPosLast = 0;
	
	
	if(*backFlag == 0)
	{
		status = 0;
		if(gBalancer1.motor.pos > backPos)
		{
			direct = -1;
			status = 0;
		}
		else if(gBalancer1.motor.pos < backPos)
		{
			direct = 1;
			status = 0;
		}
		else
		{
			direct = 0;
		}		
	}
	
	if(backPosLast != backPos)
	{
		if(gBalancer1.motor.pos > backPos)
		{
			direct = -1;
			status = 0;
		}
		else if(gBalancer1.motor.pos < backPos)
		{
			direct = 1;
			status = 0;
		}
		else
		{
			direct = 0;
		}
	}
	
	if(*backFlag == 1)
	{
		if(direct == 1)
		{
			switch(status)
			{
				case 0:
					if(gaitCnt > 30)
						vel = 350.f;
					else
						vel = 350.f;
					if(gBalancer1.motor.pos >= backPos)
					{
						vel = 0.f;
						status = 1;
					}						
					break;
				case 1:
					*backFlag = 0;
					break;
			}
		}
		else if(direct == -1)
		{
			switch(status)
			{
				case 0:
					if(gaitCnt > 30)
						vel = -350.f;
					else
						vel = -350.f;
					if(gBalancer1.motor.pos <= backPos)
					{
						vel = 0.f;
						status = 1;
					}					
					break;
				case 1:
					*backFlag = 0;
					break;
			}
		}
	
		if(gBalancer1.motor.posPulse >=GYRO1_PROTECT_MAX && vel > 0)
		{
			vel = 0.f;
		}
		else if(gBalancer1.motor.posPulse <= GYRO1_PROTECT_MIN && vel < 0)
		{
			vel = 0.f;
		}
		M3508Speed[0]=vel;
	}
	backPosLast = backPos;
}


/** 
  * @brief  陀螺复位
  * @note
  * @param  None
  * @retval None 
  */
uint8_t goBack_status = 0;
void GyroBack_x(float backPos,uint8_t *backFlag)
{
	float vel = 0.f;
	static int direct = 0,status=0;
	static float backPosLast = 0;
	
	
	if(*backFlag == 0)
	{
		status = 0;
		if(gBalancer2.motor.pos > backPos)
		{
			direct = -1;
			status = 0;
		}
		else if(gBalancer2.motor.pos < backPos)
		{
			direct = 1;
			status = 0;
		}
		else
		{
			direct = 0;
		}
	}
	
	if(backPosLast != backPos)
	{
		if(gBalancer2.motor.pos > backPos)
		{
			direct = -1;
			status = 0;
		}
		else if(gBalancer2.motor.pos < backPos)
		{
			direct = 1;
			status = 0;
		}
		else
		{
			direct = 0;
		}
	}

	
	goBack_status = status;
	if(*backFlag == 1)
	{
		if(direct == 1)
		{
			switch(status)
			{
				case 0:
					if(gaitCnt > 30)
						vel = 350.f;
					else
						vel = 350.f;
					if(gBalancer2.motor.pos >= backPos)
					{
						vel = 0.f;
						status = 1;
					}						
					break;
				case 1:
					*backFlag = 0;
					break;
			}
		}
		else if(direct == -1)
		{
			switch(status)
			{
				case 0:
					if(gaitCnt > 30)
						vel = -350.f;
					else
						vel = -350.f;
					if(gBalancer2.motor.pos <= backPos)
					{
						vel = 0.f;
						status = 1;
					}					
					break;
				case 1:
					*backFlag = 0;
					break;
			}
		}
	
		if(gBalancer2.motor.posPulse >=GYRO2_PROTECT_MAX && vel > 0)
		{
			vel = 0.f;
		}
		else if(gBalancer2.motor.posPulse <= GYRO2_PROTECT_MIN && vel < 0)
		{
			vel = 0.f;
		}		
		M3508Speed[1]=vel;
	}
	backPosLast = backPos;
}




