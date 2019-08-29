#ifndef __ADJUST_H
#define __ADJUST_H
#include "moveBase.h"

#define STEP_DISTANCE		(90.0f)
#define STEP_SIDE_DISTANCE 	(675.f)
#define ROPE_DISTANCE		(90.0f)
#define ROPE_SIDE_DISTANCE	(461.50f)
#define SLOPE_DISTANCE		(68.09f)
#define SLOPE_SIDE_DISTANCE	(1047.0f)


float StrightAdjust_RED(uint16_t gaitCnt_t,uint8_t cnt);
float StrightAdjust_BLUE(uint16_t gaitCnt_t,uint8_t cnt);
float STRAdjust_RED(uint16_t gaitCnt_t,uint8_t cnt);
float STRAdjust_BLUE(uint16_t gaitCnt_t,uint8_t cnt);

#endif



