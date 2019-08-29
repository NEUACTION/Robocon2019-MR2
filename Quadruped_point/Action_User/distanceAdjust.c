#include "distanceAdjust.h"
#include "pid.h"
#include "dataProcess.h"
#include "sensor.h"
#include "vloop.h"

float maxAngleRecord=0,minAngleRecord=0,distanceRecord=0;

extern visual_ visual;
extern leg_ legOne, legTwo, legThr, legFor;  
extern quadrupedStatus_ quadrupedStatus;

/** 
  * @brief  红场开始直线位置调整
  * @note
  * @param  None
  * @retval None
  */
float StrightAdjust_RED(uint16_t gaitCnt_t,uint8_t cnt)
{
	float ADdistance=0;
	float adjustOut=0;
	float maxAngle=0,minAngle=0;

	if(legOne.standFlag && legFor.standFlag)
		ADdistance=(12-gaitCnt_t)*240.0f;	
	else if(legTwo.standFlag && legThr.standFlag)
		ADdistance=(12-gaitCnt_t-1)*240.0f+(25-cnt)/25.0f*480.0f;	
	
	if(visual.status == 0)
	{
		maxAngle=45.0f*PI/180.0f-acosf(legThr.D.stepOneD/sqrtf(Square(legThr.pos[0]+191.4214f)+Square(legThr.pos[1])))-quadrupedStatus.euler[2];
		minAngle=45.0f*PI/180.0f-acosf(legTwo.D.stepOneD/sqrtf(Square(legTwo.pos[0]+1225.0f)+Square(legTwo.pos[1]-1033.5786f)))-quadrupedStatus.euler[2];
	}
	else if(visual.status == 1)
	{
		maxAngle=45.0f*PI/180.0f-acosf(legThr.D.stepOneD/sqrtf(Square(legThr.pos[0])+Square(legThr.pos[1]-1561.7009f)))-quadrupedStatus.euler[2];
		minAngle=45.0f*PI/180.0f-acosf(legTwo.D.stepOneD/sqrtf(Square(legTwo.pos[0])+Square(legTwo.pos[1]-100.0f)))-quadrupedStatus.euler[2];
	}
	
	if(visual.status == 0 || visual.status == 1)
		adjustOut=45.0f*PI/180.0f-acosf((legOne.D.stepOneD-182.9f)/ADdistance)-quadrupedStatus.euler[2];

	maxAngleRecord=maxAngle;
	minAngleRecord=minAngle;
	
	if(adjustOut > maxAngle)
		adjustOut=maxAngle;
	else if(adjustOut < minAngle)
		adjustOut=maxAngle;
	
	return adjustOut;
}

/** 
  * @brief  蓝场开始直线位置调整
  * @note
  * @param  None
  * @retval None
  */

float StrightAdjust_BLUE(uint16_t gaitCnt_t,uint8_t cnt)
{
	float ADdistance=0;
	float adjustOut=0;
	float maxAngle=0,minAngle=0;

	if(legTwo.standFlag && legThr.standFlag)
		ADdistance=(12-gaitCnt_t)*240.0f;	
	else if(legOne.standFlag && legFor.standFlag)
		ADdistance=(12-gaitCnt_t-1)*240.0f+(25-cnt)/25.0f*480.0f;	
	
	if(visual.status == 0)
	{
		maxAngle=adjustOut=-45.0f*PI/180.0f+acosf((legOne.D.stepOneD)/sqrtf(Square(legOne.pos[0]+1225.0f)+Square(legOne.pos[1]-1033.5786f)))-quadrupedStatus.euler[2];
		minAngle=adjustOut=-45.0f*PI/180.0f+acosf((legFor.D.stepOneD)/sqrtf(Square(legFor.pos[0]+191.4214f)+Square(legFor.pos[1])))-quadrupedStatus.euler[2];
		
	}
	else if(visual.status == 1)
	{
		maxAngle=adjustOut=-45.0f*PI/180.0f+acosf((legOne.D.stepOneD)/sqrtf(Square(legOne.pos[0])+Square(legOne.pos[1]-100.0f)))-quadrupedStatus.euler[2];
		minAngle=adjustOut=-45.0f*PI/180.0f+acosf((legFor.D.stepOneD)/sqrtf(Square(legFor.pos[0])+Square(legFor.pos[1]-1561.7009f)))-quadrupedStatus.euler[2];
	}
	
	distanceRecord=ADdistance;
	maxAngleRecord=maxAngle;
	minAngleRecord=minAngle;
	
	if(visual.status == 0 || visual.status == 1)
	{
		if(ADdistance > 3.0f && legTwo.D.stepOneD > 182.9f)
			adjustOut=-45.0f*PI/180.0f+acosf((legTwo.D.stepOneD-182.9f)/ADdistance);
		else;
	}
	
	if(adjustOut > maxAngle)
		adjustOut=maxAngle;
	else if(adjustOut < minAngle)
		adjustOut=maxAngle;
	
	adjustOut=MaxMinLimit(adjustOut,10.0f*PI/180.0f);
	
	return adjustOut;
}

/** 
* @brief  红场下台阶后到绳前的位置角度调整
  * @note
  * @param  None
  * @retval None
  */
float STRAdjust_RED(uint16_t gaitCnt_t,uint8_t cnt)
{
	float distance=0;
	float adjustOut=0;
	
	if(legOne.standFlag && legFor.standFlag)
		distance=(24-gaitCnt_t)*240.0f;	
	else if(legTwo.standFlag && legThr.standFlag)
		distance=(24-gaitCnt_t-1)*240.0f+(25-cnt)/25.0f*480.0f;	
	
	if(visual.status == 2 || visual.status == 3)
		adjustOut=acosf((legOne.D.ropeOneD-140.3f)/distance);
	
	if(adjustOut > 60.0f*PI/180)
	distanceRecord=distance;
	
	return adjustOut;
}

/** 
* @brief  蓝场下台阶后到绳前的位置角度调整
  * @note
  * @param  None
  * @retval None
  */
float STRAdjust_BLUE(uint16_t gaitCnt_t,uint8_t cnt)
{
	float distance=0;
	float adjustOut=0;
	
	distance=(24-gaitCnt_t)*240.0f+(25-cnt)/25.0f*240.0f;
	
	if(visual.status == 2 || visual.status == 3)
		adjustOut=-acosf((legTwo.D.ropeOneD-140.3f)/distance)-quadrupedStatus.euler[2];
	
	return adjustOut;
}













//绳前标准点到墙的x距离，y距离
#define WallX 140.3056f
#define RopeY 860.3056f
#define WallY 20.3056f




//计算调整的角度
//以标准点为原点，其他任意一点以此点为原点记坐标
uint8_t distanceErr=0;
float PointToPointCaculate(float leftDistance,float pointX,float pointY)
{
    static float adjustAngle=0;
    float targetDistance=0;
    //经过判断后的角度
    float angleOut=0;
    float distanceX=0;
    float distanceY=0;
    float direction=0;
    
    
    distanceX=legOne.pos[1]-WallX-pointX;
    if(visual.status==2)
        distanceY=legOne.pos[0]-RopeY-pointY;
    if(visual.status==3)
        distanceY=legOne.pos[0]-WallY-pointY;
    
    targetDistance=sqrt(distanceX*distanceX+distanceY*distanceY);
    
    if(leftDistance>targetDistance)
    {
        if(fabs((Square(0.5f*leftDistance)+Square(targetDistance)-Square(0.5f*leftDistance))/(2.0f*0.5f*leftDistance*targetDistance))<=1)
        {
            adjustAngle=acos((Square(0.5f*leftDistance)+Square(targetDistance)-Square(0.5f*leftDistance))/(2.0f*0.5f*leftDistance*targetDistance))/pi*180.0f;
            distanceErr=0;
        }else distanceErr=1;
    }
    else distanceErr=1;
    
    direction=legOne.legPos.turn+adjustAngle;
    if((distanceX+WallX)<=leftDistance*0.5f*sin(direction*pi/180.0f))
       angleOut=-adjustAngle;
    else if((distanceX+WallX)<=leftDistance*0.5f*sin(direction*pi/180.0f))
        angleOut=-adjustAngle;
    else distanceErr=1;
    
    return angleOut;   
}


//点到点距离调整
float PointTopointAdjust(uint16_t gaitCnt_t,uint8_t cnt,float pointX,float pointY)
{
    extern uint8_t receiveFlag;
    extern uint8_t ropeAdjustFlag;
    extern uint8_t distanceErr;
    //剩下的步长
    float distance=0;
	static float adjustOut=0;
    static float distanceRecord=0;
    static float angle=0;

	distance=(24-gaitCnt_t)*240.0f+(25-cnt)/25.0f*240.0f;
    
    /*采到视觉信息，产生一个三角形*/
    if(ropeAdjustFlag==1)
    {
        ropeAdjustFlag=0; 
        if((visual.status == 2 || visual.status == 3))
        {
            angle=PointToPointCaculate(distance,pointX,pointY);
            distanceRecord=distance;//记下产生这个三角形的时的距离信息
        }
    }
    //走过了一半三角形，开始换向
    if(distance<0.5f*distanceRecord)
        adjustOut=2*angle;
    
    
    else adjustOut=angle;
   
//    USART_OUT(UART4,(uint8_t *)"disad %d %d %d",(int)angle,(int)distance,distanceErr);

    return adjustOut;
}








