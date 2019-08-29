#ifndef __MODULERED_H
#define __MODULERED_H

#include "moveBase.h"
#include "sensor.h"

#define BLUE (0)
#define RED  (1)
/********结构体*********/



uint8_t StartUp_RED(void);
uint8_t Stright_RED(void);
uint8_t StepOver_RED(void);
uint8_t CrossRope_RED(void);
uint8_t STRTrans_RED(void);
uint8_t CrossRope_RED(void);
uint8_t Relay_RED(void);
uint8_t Uphill_RED(void);
void Stop(void);
void TurnMotorControl(void);
void HipAndKneeMotorControl(float lfHip,float lfKnee,float rfHip,float rfKnee,float lbHip,float lbKnee,float rbHip,float rbKnee,\
							float lfHipNext,float lfKneeNext,float rfHipNext,float rfKneeNext,float lbHipNext,float lbKneeNext,float rbHipNext,float rbKneeNext);
void Oblique(float angle);
uint16_t SpeedControl(uint16_t desirePeriod,int16_t acc,uint16_t timeCnt);
float angleTransToX(float dAngle);
float angleTransToY(float dAngle);

#endif


