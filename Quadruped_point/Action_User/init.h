#ifndef __INIT_H
#define __INIT_H

/*******************常用参数*******************/

#define RUNNING_MODE    //DEBUG_MODE

/*起始航向角度*/
#define STARTANGLE_RED 4.8f
#define STARTANGLE_BLUE 4.5f

/*台后航向角度角度*/
#define STRANGLE_RED  	48.3f
#define STRANGLE_BLUE	46.5f
 
/*过绳航向角度*/
#define ROPEANGLE_RED_2		9.0f	
#define ROPEANGLE_RED_1		9.0f

#define ROPEANGLE_BLUE_1	11.0f
#define ROPEANGLE_BLUE_2	11.0f



#ifdef RUNNING_MODE
    /*设置限幅*/
    #define START_LIMIT 	(150)
    #define GROUND_LIMIT	(4)
    #define NORMAL_LIMIT	(900)
#else
    /*设置限幅*/
    #define START_LIMIT 	(150)
    #define GROUND_LIMIT	(4)
    #define NORMAL_LIMIT	(300)

#endif



/*********************************************/

void LegParaInit(void);
void MoterInit(void);
void MotorAllOff(void);
void MotorAllStop(void);
void StartInit(float oneHip, float oneKnee, float twoHip, float twoKnee, float thrHip, float thrKnee, float forHip, float forKnee);
void ReStartInit(float oneHip, float oneKnee, float twoHip, float twoKnee, float thrHip, float thrKnee, float forHip, float forKnee);
void PosInit(void);
#endif



