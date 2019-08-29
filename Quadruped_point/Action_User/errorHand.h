#ifndef __ERROR_H
#define __ERROR_H
#include "moveBase.h"

uint8_t CanCommunicateErr(void);
void CanErrDisplay(void);
uint8_t RolloverProtect(float *euler);

uint8_t TokenErr(void);
#endif




