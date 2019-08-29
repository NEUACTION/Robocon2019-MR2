#include "dataProcess.h"
#include "sensor.h"
#include "vloop.h"
#include "cloudplatform.h"
#include "app_cfg.h"
#include "module_blue.h"
#include "module_red.h"


float sequentially_x[DATA_LENGTH]={0},sequentially_y[DATA_LENGTH]={0},timeNum[40]={0};
uint8_t adjustFlag=0;
uint8_t receiveCnt=0;
extern uint8_t site;

extern uint8_t receiveFlag;

//视觉数据
extern visual_ visual;
extern uint16_t timeRecord;
extern leg_ legOne, legTwo, legThr, legFor;
extern quadrupedControl_ quadrupedControl;
extern quadrupedStatus_ quadrupedStatus;
extern MotorType motor[4];

/**
  * @brief  数据记录,用于记录初始化姿态足端位置
  * @note
  * @param  None
  * @retval None
  */  

void DataRecord(void)
{
	legOne.readPos.hip=HipTrans(motor[legOne.ID-1].posCtrl.actualPos.hip);
	legOne.readPos.knee=KneeTrans(motor[legOne.ID-1].posCtrl.actualPos.knee);
	
	legTwo.readPos.hip=HipTrans(motor[legTwo.ID-1].posCtrl.actualPos.hip);
	legTwo.readPos.knee=KneeTrans(motor[legTwo.ID-1].posCtrl.actualPos.knee);
	
	legThr.readPos.hip=HipTrans(motor[legThr.ID-1].posCtrl.actualPos.hip);
	legThr.readPos.knee=KneeTrans(motor[legThr.ID-1].posCtrl.actualPos.knee);
	
	legFor.readPos.hip=HipTrans(motor[legFor.ID-1].posCtrl.actualPos.hip);
	legFor.readPos.knee=KneeTrans(motor[legFor.ID-1].posCtrl.actualPos.knee);
	
	GetFootEndCoordinate_B(&legOne);
	GetFootEndCoordinate_B(&legTwo);
	GetFootEndCoordinate_B(&legThr);
	GetFootEndCoordinate_B(&legFor);
	
	legOne.worldLast.x=legOne.world.x; legOne.worldLast.y=legOne.world.y; legOne.worldLast.z=legOne.world.z;
	legTwo.worldLast.x=legTwo.world.x; legTwo.worldLast.y=legTwo.world.y; legTwo.worldLast.z=legTwo.world.z;
	legThr.worldLast.x=legThr.world.x; legThr.worldLast.y=legThr.world.y; legThr.worldLast.z=legThr.world.z;
	legFor.worldLast.x=legFor.world.x; legFor.worldLast.y=legFor.world.y; legFor.worldLast.z=legFor.world.z;
	
}

/**
  * @brief  数据转化
  * @note
  * @param  None
  * @retval None
  */   
void DataConversion(void)
{
	legOne.readPos.hip=HipTrans(motor[legOne.ID-1].posCtrl.actualPos.hip);
	legOne.readPos.knee=KneeTrans(motor[legOne.ID-1].posCtrl.actualPos.knee);
	
	legTwo.readPos.hip=HipTrans(motor[legTwo.ID-1].posCtrl.actualPos.hip);
	legTwo.readPos.knee=KneeTrans(motor[legTwo.ID-1].posCtrl.actualPos.knee);
	
	legThr.readPos.hip=HipTrans(motor[legThr.ID-1].posCtrl.actualPos.hip);
	legThr.readPos.knee=KneeTrans(motor[legThr.ID-1].posCtrl.actualPos.knee);
	
	legFor.readPos.hip=HipTrans(motor[legFor.ID-1].posCtrl.actualPos.hip);
	legFor.readPos.knee=KneeTrans(motor[legFor.ID-1].posCtrl.actualPos.knee);
	
	/*得到足端相对于肩关节的位置*/
	GetFootEndCoordinate_B(&legOne);
	GetFootEndCoordinate_B(&legTwo);
	GetFootEndCoordinate_B(&legThr);
	GetFootEndCoordinate_B(&legFor);
	Compensate();
}


/**
  * @brief  视觉信息补偿
  * @note
  * @param  None
  * @retval None
  */
void Compensate(void)
{
	uint8_t pointNum=0;
	float x_distance=0,y_distance=0;
	float timeSum=0;
	float dx=0,dy=0,angle=0;
	float addX=0,addY=0;
	
	if(legOne.standFlag && legFor.standFlag)
	{
		dx=-(legOne.world.x-legOne.worldLast.x+legFor.world.x-legFor.worldLast.x)/2.0f;
		dy=-(legOne.world.y-legOne.worldLast.y+legFor.world.y-legFor.worldLast.y)/2.0f;
		angle=atan2f(dy,dx);
	}
	else
	{
		dx=-(legTwo.world.x-legTwo.worldLast.x+legThr.world.x-legThr.worldLast.x)/2.0f;
		dy=-(legTwo.world.y-legTwo.worldLast.y+legThr.world.y-legThr.worldLast.y)/2.0f;
		angle=atan2f(dy,dx);
	}
	
	if(receiveCnt < DATA_LENGTH)
	{
		sequentially_x[receiveCnt]=sqrt(dx*dx+dy*dy)*cosf(angle+quadrupedStatus.euler[2]);
		sequentially_y[receiveCnt]=sqrt(dx*dx+dy*dy)*sinf(angle+quadrupedStatus.euler[2]);
		timeNum[receiveCnt]=timeRecord/10;
		receiveCnt++;
	}
	else if(receiveCnt == DATA_LENGTH)
	{
		for(uint8_t i=0;i<= DATA_LENGTH-2;i++)
		{
			sequentially_x[i]=sequentially_x[i+1];
			sequentially_y[i]=sequentially_y[i+1];
			timeNum[i]=timeNum[i+1];
		}
		
		sequentially_x[receiveCnt-1]=sqrt(dx*dx+dy*dy)*cosf(angle+quadrupedStatus.euler[2]);
		sequentially_y[receiveCnt-1]=sqrt(dx*dx+dy*dy)*sinf(angle+quadrupedStatus.euler[2]);
		timeNum[receiveCnt-1]=timeRecord/10;	
	}
		
	addX=sqrt(dx*dx+dy*dy)*cosf(angle+quadrupedStatus.euler[2]);
	addY=sqrt(dx*dx+dy*dy)*sinf(angle+quadrupedStatus.euler[2]);
	
	/*根据腿的运动估计车身的位置，用来判断视觉信息*/
	JudgeInformation(addX, addY);
	
	/*接收到视觉信息*/
	if(receiveFlag == 1)
	{
		receiveFlag=0;
		for(uint8_t j=0;((timeSum+=timeNum[j])<visual.time)&&(j<=DATA_LENGTH-1);j++)
		{
			pointNum=j ;
		}
		for(uint8_t k=0;k<=pointNum;k++)
		{
			x_distance+=sequentially_x[receiveCnt-k-1];
			y_distance+=sequentially_y[receiveCnt-k-1];
		}
		
		visual.compensate[0]=x_distance;
		visual.compensate[1]=y_distance;
		
		if(site == RED)
		{
			DistanceConversion_RED();
			GetLegRD_RED(&legOne);
			GetLegRD_RED(&legTwo);
			GetLegRD_RED(&legThr);
			GetLegRD_RED(&legFor);
		}
		else
		{
			DistanceConversion_BLUE();
			GetLegRD_BLUE(&legOne);
			GetLegRD_BLUE(&legTwo);
			GetLegRD_BLUE(&legThr);
			GetLegRD_BLUE(&legFor);
		}
		adjustFlag=1;
//		if(visual.status  == 0 || visual.status  == 2 || visual.status  == 3  || visual.status  == 4)
//		{
//			/*视觉信息与估计位置差100mm以内，判断视觉信息有效*/
//			if(fabsf(quadrupedStatus.estimatedPos[0]-quadrupedStatus.position[0])<100.0f && fabsf(quadrupedStatus.estimatedPos[1]-quadrupedStatus.position[1])<100.0f)
//			{
//				quadrupedStatus.estimatedPos[0]=quadrupedStatus.position[0];
//				quadrupedStatus.estimatedPos[1]=quadrupedStatus.position[1];
//				adjustFlag=1;
//			}
//			else
//				adjustFlag=0;
//		}
//		else if(visual.status  == 1)
//		{
//			if(fabsf(quadrupedStatus.estimatedPos[0]-quadrupedStatus.position[0])<100.0f)
//			{
//				quadrupedStatus.estimatedPos[0]=quadrupedStatus.position[0];
//				adjustFlag=1;
//			}
//			else
//				adjustFlag=0;
//		}
	}
	
	visual.statusLast=visual.status;
	legOne.worldLast.x=legOne.world.x; legOne.worldLast.y=legOne.world.y; legOne.worldLast.z=legOne.world.z;
	legTwo.worldLast.x=legTwo.world.x; legTwo.worldLast.y=legTwo.world.y; legTwo.worldLast.z=legTwo.world.z;
	legThr.worldLast.x=legThr.world.x; legThr.worldLast.y=legThr.world.y; legThr.worldLast.z=legThr.world.z;
	legFor.worldLast.x=legFor.world.x; legFor.worldLast.y=legFor.world.y; legFor.worldLast.z=legFor.world.z;
	
}

/**
  * @brief  红场视觉信息转换
  * @note
  * @param  None
  * @retval None
  */
extern float CPPos;
void DistanceConversion_RED(void)
{
	float cameraYaw=0;
	float angle=0,length=0;
	
	angle=atan2f(visual.compensate[1],visual.compensate[0]);
	length=sqrt(Square(visual.compensate[1])+Square(visual.compensate[0]));
	
	cameraYaw=CPPos*PI/180.0f;
	switch(visual.status)
	{
		case 0:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(visual.recordYawLast)+length*cosf(angle); 
			break;
		case 1:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(-45.0f*PI/180.0f+cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(45.0f*PI/180.0f+visual.recordYawLast)-length*sinf(-45.0f*PI/180.0f+angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(-45.0f*PI/180.0f+cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(45.0f*PI/180.0f+visual.recordYawLast)-length*cosf(-45.0f*PI/180.0f+angle);
			break;
		case 2:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(visual.recordYawLast)+length*cosf(angle); 
			break;
		case 3:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(visual.recordYawLast)+length*cosf(angle); 
			break;
		case 4: 
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)-LEG_SPECE/2.0f*sinf(visual.recordYawLast)-length*cosf(angle); 
			break; 
		case 5:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=0.0f;
			break;
		case 6:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)-LEG_SPECE/2.0f*sinf(visual.recordYawLast)-length*cosf(angle);  
			break;
		case 7:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)-LEG_SPECE/2.0f*sinf(visual.recordYawLast)-length*cosf(angle); 
			break;
		default:
			break;
		}
		quadrupedStatus.position[0]=visual.distanceNow[0];
		quadrupedStatus.position[1]=visual.distanceNow[1];
}

/**
  * @brief  蓝场视觉信息转换
  * @note
  * @param  None
  * @retval None
  */
void DistanceConversion_BLUE(void)
{
	float cameraYaw=0;
	float angle=0,length=0;
	angle=atan2f(visual.compensate[1],visual.compensate[0]);
	length=sqrt(Square(visual.compensate[1])+Square(visual.compensate[0]));
	
	cameraYaw=CPPos*PI/180.0f;
	switch(visual.status)
	{
		case 0:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)-LEG_SPECE/2.0f*sinf(visual.recordYawLast)-length*cosf(angle); 
			break;
		case 1:
			visual.distanceNow[0]=visual.distance_C[0]-CAMERA_DISTANCE*sinf(-45.0f*PI/180.0f+cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(45.0f*PI/180.0f+visual.recordYawLast)-length*cosf(-45.0f*PI/180.0f+angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*cosf(-45.0f*PI/180.0f+cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(45.0f*PI/180.0f+visual.recordYawLast)-length*sinf(-45.0f*PI/180.0f+angle);
			break;
		case 2:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)-LEG_SPECE/2.0f*sinf(visual.recordYawLast)-length*cosf(angle); 
			break;
		case 3:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]-CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)-LEG_SPECE/2.0f*sinf(visual.recordYawLast)-length*cosf(angle); 
			break;
		case 4: 
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(visual.recordYawLast)+length*cosf(angle); 
			break;
		case 5:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=0.0f;
			break;
		case 6:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(visual.recordYawLast)+length*cosf(angle); 
			break;
		case 7:
			visual.distanceNow[0]=visual.distance_C[0]+CAMERA_DISTANCE*cosf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*cosf(visual.recordYawLast)-length*sinf(angle);
			visual.distanceNow[1]=visual.distance_C[1]+CAMERA_DISTANCE*sinf(cameraYaw+visual.recordYawLast)+LEG_SPECE/2.0f*sinf(visual.recordYawLast)+length*cosf(angle); 
			break;
		default:
			break;
	}
	quadrupedStatus.position[0]=visual.distanceNow[0];
	quadrupedStatus.position[1]=visual.distanceNow[1];
}


/**
  * @brief  得到腿相对障碍物的距离
  * @note 
  * @param  None
  * @retval None
  */
void GetLegRD_RED(leg_ *leg)
{
	float angle=0,length=0;
	angle=atan2f((*leg).world.y,(*leg).world.x);
	length=sqrt(Square((*leg).world.y)+Square((*leg).world.x));
	
	switch(visual.status)
	{
		case 0:
			/*先得到足端的位置*/
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle);
			
			/*再得到足端到台的位置*/
			(*leg).D.stepOneD=((*leg).pos[0]+(*leg).pos[1]+191.4214f)/sqrtf(2.0f);
			(*leg).D.stepTwoD=(*leg).D.stepOneD+300.0f;
			break;
		case 1:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(-45.0f*PI/180.0f+angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(-45.0f*PI/180.0f+angle);
		
			(*leg).D.stepOneD=(*leg).pos[0];
			(*leg).D.stepTwoD=(*leg).D.stepOneD+300.0f;
			break;
		case 2:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle);
			
			(*leg).D.stepOneD=((*leg).pos[0]+(*leg).pos[1]-3808.58f)/sqrtf(2.0f);
			(*leg).D.stepTwoD=(*leg).D.stepOneD+300.0f;
		
			(*leg).D.ropeTwoD=(*leg).pos[0]+880.0f;
			(*leg).D.ropeOneD=(*leg).D.ropeTwoD-760.0f;
			break;
		case 3:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle);
			
			(*leg).D.ropeTwoD=(*leg).pos[0];
			(*leg).D.ropeOneD=(*leg).D.ropeTwoD-760.0f;
			break;
		case 4:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle);
			
			(*leg).D.underSlopeD=(*leg).pos[1];
			break;
		case 5:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle);
		
			break;
		case 6:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle);
			
			(*leg).D.upSlopeD=(*leg).pos[1];
			break;
		case 7:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle);
			
			(*leg).D.ending=(*leg).pos[1];
			break;
		default:
			break;
	}
}

/**
  * @brief  蓝场得到腿相对障碍物的距离
  * @note 
  * @param  None
  * @retval None
  */
void GetLegRD_BLUE(leg_ *leg)
{
	float angle=0,length=0;
	angle=atan2f((*leg).world.y,(*leg).world.x);
	length=sqrt(Square((*leg).world.y)+Square((*leg).world.x));
	
	switch(visual.status)
	{
		case 0:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle); 
		
			(*leg).D.stepOneD=((*leg).pos[0]+(*leg).pos[1]+191.4214f)/sqrtf(2.0f);
			(*leg).D.stepTwoD=(*leg).D.stepOneD+300.0f;
			break;
		case 1:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*cosf(-45.0f*PI/180.0f+angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*sinf(-45.0f*PI/180.0f+angle);
		
			(*leg).D.stepOneD=(*leg).pos[0];
			(*leg).D.stepTwoD=(*leg).D.stepOneD+300.0f;
			break;
		case 2:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle); 
			
			(*leg).D.stepOneD=((*leg).pos[0]+(*leg).pos[1]-3808.58f)/sqrtf(2.0f);
			(*leg).D.stepTwoD=(*leg).D.stepOneD+300.0f;
		
			(*leg).D.ropeTwoD=(*leg).pos[0]+880.0f;
			(*leg).D.ropeOneD=(*leg).D.ropeTwoD-760.0f;
			break;
		case 3:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]-length*cosf(angle); 
			
			(*leg).D.ropeTwoD=(*leg).pos[0];
			(*leg).D.ropeOneD=(*leg).D.ropeTwoD-760.0f;
			break;
		case 4:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle); 
			
			(*leg).D.underSlopeD=(*leg).pos[1];
			break;
		case 5:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle); 
		
			break;
		case 6:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle); 
			
			(*leg).D.upSlopeD=(*leg).pos[1];
			break;
		case 7:
			(*leg).pos[0]=quadrupedStatus.position[0]-length*sinf(angle);
			(*leg).pos[1]=quadrupedStatus.position[1]+length*cosf(angle); 
			
			(*leg).D.ending=(*leg).pos[1];
			break;
		default:
			break;
	}
}





/**
  * @brief  得到腿相对身体中心的距离
  * @note
  * @param  None
  * @retval None
  */
void GetFootEndCoordinate_B(leg_ *leg)
{
	(*leg).read.x = -(L0*sinf(-(*leg).readPos.hip) + L1 * sinf(-(*leg).readPos.hip + (*leg).readPos.knee))*sinf((*leg).readPos.turn);
	(*leg).read.y = (L0*sinf(-(*leg).readPos.hip) + L1 * sinf(-(*leg).readPos.hip + (*leg).readPos.knee))*cosf((*leg).readPos.turn);
	(*leg).read.z = -(L0*cosf(-(*leg).readPos.hip) + L1 * cosf(-(*leg).readPos.hip + (*leg).readPos.knee));
	
	(*leg).world=RCT((*leg),quadrupedStatus.euler);
}

/**
  * @brief  由机体坐标系到世界坐标系
  * @note   None
  * @param  None
  * @param  None
  * @retval None
  */
coordinate RCT(leg_ leg, float *w)
{
	coordinate worldCoordinate;
	
	worldCoordinate.x = cosf(w[2])*cosf(w[1])*(leg.read.x+leg.rePos.x) + (-sinf(w[2])*cosf(w[0]) + cosf(w[2])*sinf(w[0])*sinf(w[1]))*(leg.read.y+leg.rePos.y)+(sinf(w[2])*sinf(w[0]) + cosf(w[2])*sinf(w[1])*cosf(w[0]))*(leg.read.z+leg.rePos.z);
	
	worldCoordinate.y = sinf(w[2])*cosf(w[1])*(leg.read.x+leg.rePos.x) + (cosf(w[2])*cosf(w[0]) + sinf(w[2])*sinf(w[1])*sinf(w[0]))*(leg.read.y+leg.rePos.y) + (-cosf(w[2])*sinf(w[0]) + sinf(w[2])*sinf(w[1])*cosf(w[0]))*(leg.read.z+leg.rePos.z);
	
	worldCoordinate.z = -sinf(w[1])*(leg.read.x+leg.rePos.x) +  cosf(w[1])*sinf(w[0])*(leg.read.y+leg.rePos.y) + cosf(w[1])*cosf(w[0])*(leg.read.z+leg.rePos.z);
	
	return worldCoordinate;
}







/*********************************************************/




