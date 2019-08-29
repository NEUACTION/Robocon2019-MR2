/**
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H


#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <elmo.h>
#include <usart.h>
#include <app_cfg.h>
#include "usart.h" 
#include "timer.h"

/* Includes ------------------------------------------------------------------*/



/* Exported types ------------------------------------------------------------*/

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

/******************************define*************************************/

//运行周期

#define DESIRE_ANGLE	(0.5*PI/180.0f)

#define PI 				(3.1415926f)
/*大腿长度*/
#define L0 				(230.0f)
/*小腿长度*/
#define L1 				(200.0f)

/*腿间距*/
#define LEG_SPECE 		(450.0f)

#define SWING_MODE 		0	
#define SUPPORT_MODE 	1

#define LEFT  	0
#define RIGHT 	1
//id号
//左前腿（1号腿）
#define LF_LEG  1		

//右前腿（2号腿）
#define RF_LEG	2

//左后腿（3号腿）
#define LB_LEG	3

//右后腿（4号腿）
#define RB_LEG	4

//抬升电机
#define LIFT_UP_ID	5

#define GYRO1       6
#define GYRO2       7




//2006转一周脉冲都为8192
#define COUNTS_PER_ROUND	(8192.0f)
//2006电机减速比，相当于给出去的脉冲要多乘上减速比
#define REDUCTION_RATIO		(36.0f)
//转向关节减速比
#define TURN_RATIO			(48.0f/17.0f)


//terrain选择
#define START				(0)
#define FLAT  				(1)
#define STEP  				(2)
#define STEP_TO_ROPE		(3)
#define ROPE  				(4)
#define RELAY 				(5)
#define UPHILL				(6)
#define STOP				(7)
/**
  * @}
  */
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

typedef struct {
	float x;
	float y;
	float z; 
}coordinate;

typedef struct {
	float a;
	float b;
	float c;

}para;

typedef struct{
	float turn;
	float hip;
	float knee;
    float lift;
}joint;

typedef struct{
	uint16_t TURN_ID;
	uint16_t HIP_ID;
	uint16_t KNEE_ID;
}id;

typedef struct{
	float support_S;
	float swing_body_S;
	float swing_world_S;
	float supportAngle;
	float swing_body_Angle;
	float swing_world_Angle;
	float support_speed;
}trajectory_;

typedef struct{
	float stepOneD;
	float stepTwoD;
	float stepSideD;
	float ropeOneD;
	float ropeTwoD;
	float ropeSideD;
	float underSlopeD;
	float upSlopeD;
	float ending;
}distance;

typedef struct {

	distance D;
	uint8_t standFlag;
	uint8_t ID;
	coordinate worldLast;
	coordinate rePos;
	coordinate read;
	coordinate world;
	joint readCur;
	joint legPos;
	joint legNextPos;
	joint readPos;
	float pos[2];
    float ropeTurn;
}leg_;

typedef struct{
	float euler[3];
	float gyro[3];
	float position[2];
	float estimatedPos[2];
}quadrupedStatus_;

typedef struct{
	float posAngle;
	float yawAdjust;
	float legAngle;
	float lf_rbAngleAdjust;
	float rf_lbAngleAdjust;
    int8_t lf_rbForward;
    int8_t rf_lbForward;
    
}quadrupedControl_;




void walk_RED(void);
void walk_BLUE(void);
void SpeedPlan(uint16_t *timeLast,uint16_t *timeCnt);
float Square(float num);
float Cubed(float num);
#endif /* ___H */

/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/

