#ifndef DEFINE_H_
#define DEFINE_H_
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "vloop.h"
#include <ucos_ii.h>
#include "app_cfg.h"
#define LF  1
#define RF	2
#define LH	3
#define RH	4
#define SWING 0
#define SUPPORT 1




#define pi      3.1415926535f
#define CmdPeriod 1 //给转接板发指令的时间
#define THIGH 230.0f		//大腿长度
#define CRUS  210.0f		//小腿长度
#define BODY_LENGTH 250.0f //身体长度
#define BODY_WIDETH 250.0f //身体宽度
#define BODY_THICK 20.0f

#define LFCOAXE 1
#define LFSHOULDER 2
#define LFKNEE 3
#define RFCOAXE 4
#define RFSHOULDER 5
#define RFKNEE 6
#define LHCOAXE 7
#define LHSHOULDER 8
#define LHKNEE 9
#define RHCOAXE 10
#define RHSHOULDER 11
#define RHKNEE 12



//parama about vloop and posloop
		//最大速度
#define  VEL_KP_3508   			 0.1f			  //速度环Kp
#define  VEL_KI_3508				 0.001f			//速度环Ki
#define  POS_KP_3508         0.11f
#define  POS_KD_3508         0.0f

#define  VEL_MAX_2006				 2400.0f			//最大速度
#define  VEL_KP_2006   			 0.05f			//速度环Kp
#define  VEL_KI_2006				 0.003f			//速度环Ki
#define  POS_KP_2006         0.027f
#define  POS_KD_2006         0.05f





#define ROLLKP_RFLH 													0.8f/360.0f*2*3.141593
#define ROLLKP_LFRH 													0.6f

#define PITCHKP_RFLH 													0.6f
#define PITCHKP_LFRH 													0.8f/360.0f*2*3.141593

#define YAWLKP 												1.0f/360.0f*2*3.141593
//#define ROLLKP_F 												0.1f

typedef struct {
	double pitch;
	double yawl;
	double roll;
}orientation;
typedef struct  {
	double X;
	double Y;
	double Z;

}coordinate_;
typedef struct  
 {
	 orientation orientation;
	 coordinate_ inverted;
	 double Y;
	 double Z;
	 double X;
 }body_;
 
 
 typedef struct {
	double X;
	double Y;
	double Z;
	 coordinate_ desiredPos;
 }endeffector;

 typedef struct
{
	float actual;
	float desired;
	float err;
	
}PID;
typedef struct 
{
	float haa;
	float detaHight;
	float balanceHeight;
}PIDout;
 
 
 
extern endeffector EE[4];

 
 //读电机位置
 typedef struct{
	coordinate_ realEEPos;
//	leg_ joint;
	float vel;
 }read_motor;

  


//联合体数据
typedef union
{
	uint8_t data8[4];	
	int16_t data16[2];	
	int data32;
	float dataf;
	
}UnionDataType;








#endif 
