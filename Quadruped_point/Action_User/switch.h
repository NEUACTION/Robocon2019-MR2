#ifndef __SWITCH_H
#define __SWITCH_H
#include "gpio.h"
#include "stdint.h"

/*选择红蓝场*/
#define SELECT_SITE  			(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_7))

#define GO						(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14))

#define WARMUP                 	(GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_1))

typedef struct
{
    uint8_t button;
    uint8_t lastButton;
    uint8_t statusButton1;
    uint8_t statusButton2;
    uint8_t statusButton3;
    uint8_t statusButton4;
    uint8_t status;
    uint8_t lastStatus;
    uint8_t restart;
    uint8_t restartOver;
    uint8_t wait;
    uint8_t waitKey;
    uint8_t waitLight;
    uint8_t waitCnt;
    
    uint8_t lightSwitch;
    float liftPos;
}reset_;




void SwitchInit(void);
uint8_t Manualtrigger(void);

void GyroSelfCheck(void);
void readGPIO(void);
void DataCleanUp(void);
void motorReInit(void);
void RobotReset(void);
void ResetStatusSelect(void);
void posReinit(void);
void resetInit(void);
void ResetDataOut(void);
void ResetScan(void);
#endif

