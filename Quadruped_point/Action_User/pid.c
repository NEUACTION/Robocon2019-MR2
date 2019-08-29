#include "pid.h"
#include "moveBase.h"
#include "sensor.h"
#include "vloop.h"
#include "balance.h"

/*距离pid参数结构体*/
pid_t distanceController;

/*航向pid参数结构体*/
pid_t yawController;

/*1、3号腿支撑时陀螺闭环参数*/
gyro_t lf_rb;

/*2、4号腿支撑时陀螺闭环参数*/
gyro_t rf_lb;

extern leg_ legOne, legTwo, legThr, legFor;  
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern visual_ visual;
/*
串行陀螺控制器
*/

Serial_ serial1;
Serial_ serial2;


void sreialPIDInit(void)
{
    serial1.insideKp=20.f;
    serial1.outsideKp=12.0f;
    serial1.max1=40;
    serial1.max2=400;
    
    serial2.insideKp=20.f;
    serial2.outsideKp=20.0f;
    serial2.max1=40;
    serial2.max2=400;
    
}

/** 
  * @brief  pid参数初始化
  * @note
  * @param  None
  * @retval None 
  */
void PIDParaInit(void)
{
	
	
	distanceController.kp=0.3f*PI/180.0f;
	distanceController.ki=0.0f;
	distanceController.kd=0.0f;
	distanceController.integralLimit=8.0f*PI/180.0f;
	
	yawController.kp=1.8f;
	yawController.ki=0.0f;
	yawController.kd=0.0f;
	yawController.integralLimit=10.0f*PI/180.0f;
	
	lf_rb.k=2600.0f;
	lf_rb.b=650.0f;
	lf_rb.limit=300.0f;
	
	rf_lb.k=2600.0f;
	rf_lb.b=650.0f;
	rf_lb.limit=300.0f;
	
}
/** 
  * @brief  距离pid
  * @note
  * @param  None
  * @retval None 
  */
float DistanceControl(float posErr)
{
	float distanceErr=0;
	float pidOut=0;
	
	distanceErr=posErr;
	
	distanceController.integral+=distanceErr;
	distanceController.integral=MaxMinLimit(distanceController.integral,distanceController.integralLimit);
	
	pidOut=distanceController.kp*distanceErr+distanceController.ki*distanceController.integral+distanceController.kd*(distanceErr-distanceController.errRecord);
	
	pidOut=MaxMinLimit(pidOut,20.0f*PI/180.0f);    
	
	distanceController.errRecord=distanceErr;
	return pidOut;
}


/** 
  * @brief  航向角度pid
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
float YawControl(float actulYaw,float desireYaw)
{
	float yawErr = 0;
	float pidOut = 0;
	
	yawErr = desireYaw - actulYaw;
	if(yawErr > 0.5f*PI/180.0f || yawErr < -0.5f*PI/180.0f)
	{
		yawController.integral+=yawErr;
		yawController.integral=MaxMinLimit(yawController.integral,yawController.integralLimit);
		
		pidOut = yawController.kp*yawErr+yawController.ki*(yawController.integral)+yawController.kd*(yawErr-yawController.errRecord);
		
		pidOut=MaxMinLimit(pidOut,10.0f*PI/180.0f);  
			
	}
	else 
		pidOut=0;
	
	yawController.errRecord=yawErr;	
	return pidOut;	
}

/** 
  * @brief  通过角度改时间
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
int16_t timeControl(float desireX,float desireY)
{
	int16_t timeAddOut=0;
	float kp=80.0f;
	if(legOne.standFlag && legFor.standFlag)
	{
		if(fabsf(quadrupedStatus.euler[0]-desireX) < 0.3f*PI/180.0f)
			timeAddOut=0;
		else
			timeAddOut=kp*(quadrupedStatus.euler[0]-desireX);
	}
	else
	{
		if(fabsf(quadrupedStatus.euler[1]-desireY) < 0.3f*PI/180.0f)
			timeAddOut=0;
		else
			timeAddOut=kp*(quadrupedStatus.euler[1]-desireY);
	}
	if(timeAddOut > 5)
		timeAddOut=5;
	else if(timeAddOut < -5)
		timeAddOut=-5;

	
	return timeAddOut;
}


/** 
  * @brief  左前右后轴身体姿态角pid
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
float LF_RBAngleControl(float dAngle,float actulAngle,float actulGyro)
{
	extern BalancerType gBalancer2;
	float velOut=0;
	if(legOne.standFlag && legFor.standFlag)
	{

        /*外环角度偏差*/
        serial2.angleErr=dAngle-gBalancer2.sysPos.angle[0];
        
        /*外环角度增益输出角速度*/
        serial2.angleLoopOut=serial2.angleErr*serial2.outsideKp;
        
        /*外环角速度输入*/
        serial2.wLoopIn=MaxMinLimit(serial2.angleLoopOut,serial2.max1);
        
        /*内环角速度 与外环输出做差*/
        serial2.wErr=serial2.wLoopIn-gBalancer2.sysPos.angularVelocity[0];
        
        /*内环最终输出*/
        serial2.outPut=serial2.wErr*serial2.insideKp;
        
		velOut=MaxMinLimit(serial2.outPut,lf_rb.limit);
	}
	else;
	return velOut;
}




/** 
  * @brief  右前左后轴身体姿态角pid
  * @note
  * @param  None
  * @param  None
  * @retval None
  */
float RF_LBAngleControl(float dAngle,float actulAngle,float actulGyro)
{
	extern BalancerType gBalancer1;

	float velOut=0;
	
	if(legTwo.standFlag && legThr.standFlag)
	{

        /*外环角度偏差*/
        serial1.angleErr=dAngle-gBalancer1.sysPos.angle[1];
        
        /*外环角度增益输出角速度*/
        serial1.angleLoopOut=serial1.angleErr*serial1.outsideKp;
        
        /*外环角速度输入*/
        serial1.wLoopIn=MaxMinLimit(serial1.angleLoopOut,serial1.max1);

        
        /*内环角速度 与外环输出做差*/
        serial1.wErr=serial1.wLoopIn-gBalancer1.sysPos.angularVelocity[1];

        /*内环最终输出*/
        serial1.outPut=serial1.wErr*serial1.insideKp;

		velOut=MaxMinLimit(serial1.outPut,rf_lb.limit);
	}
	else;
	return velOut;
}

