#ifndef __LIFT_H
#define __LIFT_H
#include "moveBase.h"


typedef struct
{
    uint8_t cnt;
    uint8_t delaytime;
    uint8_t lightSignal;
    uint8_t liftFlag;
    int pos;
}lift_;


void liftInit(void);
uint8_t Lift(void);



#endif




