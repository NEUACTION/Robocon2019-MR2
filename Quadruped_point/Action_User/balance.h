#ifndef __BALANCE_H
#define __BALANCE_H
#include <stdint.h>

/*平衡陀螺状态*/
#define     RESET_STATUS    100
#define     WAIT_STATUS_1   201
#define	    WAIT_STATUS_2   202
#define     RUNNING_STATUS  301
#define     ERROR_STATUS    401
#define     CHECK_TEST      110
#define     TEST_STATUS		801
#define     TEST_STATUS_2	802
#define     TEST_STATUS_3	803
#define     NORMAL          1 





#define POSITIVE    1
#define NEGATIVE   -1
#define GYRO_DIRECTION 	POSITIVE    //角速度方向垂直向上

#define GYRO1_PROTECT_MAX	(42.0f+30.0f)/360.f*19.2f*8192.0f*3.0f      //76.3f
#define GYRO1_PROTECT_MIN	(42.0f-30.0f)/360.f*19.2f*8192.0f*3.0f       //7.63f
#define GYRO2_PROTECT_MAX	(42.0f+30.0f)/360.f*19.2f*8192.0f*3.0f
#define GYRO2_PROTECT_MIN	(42.0f-30.0f)/360.f*19.2f*8192.0f*3.0f

#define GEAR_RATIO				3.f  //1 : 3
#define M3508_RATIO				19.2f // 1:19.2


//各角度单位变换式
#define RAD_TO_PULSE(rad)      (rad * 1304.458f) //  8192 / 6.28 
#define RAD_TO_DEGREE(rad)     (rad * 57.325f)  //   360 / 6.28
#define PULSE_TO_RAD(pulse)    (pulse * 0.0007666f)
#define PULSE_TO_DEGREE(pulse) (pulse * 0.0439f)
#define DEGREE_TO_RAD(degree)  (degree * 0.017444f)
#define DEGREE_TO_PULSE(degree) (degree * 27.7556f)

//脉冲转陀螺滚转角角度
#define PULSE_TO_BALANCER_DEGREE(pulse)  (PULSE_TO_DEGREE((pulse / GEAR_RATIO / M3508_RATIO)))//转换为陀螺的进动角/度
#define BALANCER_DEGREE_TO_PULSE(degree)  (DEGREE_TO_PULSE((degree * GEAR_RATIO * M3508_RATIO)))//转换为陀螺的进动角/度
#define  MAX_GYRO_POS (17.f)
#define  MIN_GYRO_POS (-22.3f)
#define ZERO_GYRO_POS (0.1f)          //(3.44f)//(-1.83f)

#define ZERO_ENCODER_POS (41.96f)
#define START_ENCODER_POS (42.0f+10.0f)

//转速：pwmRank = 12   17000rpm  sunny_sky

//在主控和陀螺仪端看过去，顺时针陀螺仪为负（a），逆时针陀螺仪为正（b）
//朝着3508的输出端，顺时针电机往增大(1)，逆时针电机往减小(-1)
//a-1  b1  陀螺顺时针旋转的正确调节方向
//a1  b-1  陀螺逆时针旋转的正确调节方向

typedef struct
{
  float angle[3];
	
	float angularVelocity[3];
  
 	//用来检查定位系统的时间间隔
	int time;

}SysPosType;


typedef struct
{

	float kp;
	float ki;
	float kd;

}PIDType;


typedef struct
{
	float speed;
	int speedPulse;
	float pos;
	int posPulse;
	float current;
	int currentPulse;
}BalanceMotorType;

typedef struct 
{	
	float rollAngle;
	
	float rollAngleSpeed;
	
	BalanceMotorType motor;
	 
  SysPosType sysPos;
	
	PIDType gyroPid;
	
	PIDType encoderPid;
	
	PIDType forcePid;
		
	int runTime;

	int runStatus;
	
	float gyroZeroPos;
	
	float TLE5012_RollAngle;
    
    uint8_t cnt;
}BalancerType;

void BalancerControl6(float w1);
void BalancerControl7(float w1);

void GyroProtect(int* speedPulse);
void SpeedFilter(float speed);
void Balance(void);
void BalanceFullState1(void);
void BalanceFullState2(void);
void balanceCtrlInit(void);

void BalanceOpenLoopControl_y(void);
void BalanceOpenLoopControl_x(void);
float CalculBalancer1Dis(float openLoopPeriod, float openLoopVel);
float CalculBalancer2Dis(float openLoopPeriod, float openLoopVel);
int GoToOpenLoopInitPos1(float zeroMotorPos1, float openLoopDis);
int GoToOpenLoopInitPos2(float zeroMotorPos2, float openLoopDis);
void GyroBack_y(float backPos,uint8_t *backFlag);
void GyroBack_x(float backPos,uint8_t *backFlag);
void SerialControl(uint8_t axis);
void BalancerMotorControl(float w,uint8_t ID);//
#endif





