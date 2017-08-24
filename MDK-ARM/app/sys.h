/********************************************************************************************************\
*                                     DJI System Global Macros Definations
*
*                                   (c) Copyright 2015; Dji, Inc.
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
\********************************************************************************************************/

#ifndef __SYS_H__
#define __SYS_H__

#include "can.h"
#include "mytype.h"
#include "usart.h"

#define FIRMWARE_VERSION #MAIN_VERSION##'.'
#define MAIN_VERSION 1
#define SUB_VERSION1 0
#define SUB_VERSION2 0


#define CHASSIS_CAN       hcan1
#define ZGYRO_CAN         hcan2
#define CHASSIS_ZGYRO_CAN hcan1
#define GIMBAL_CAN        hcan1

#define DBUS_HUART  huart1 //for dji remote controler reciever
#define JUDGE_HUART huart3 //connected to judge system
#define CV_HUART    huart6 //connected to manifold/TXone

#define VAL_LIMIT(val, min, max)\
if(val<=min)\
{\
	val = min;\
}\
else if(val>=max)\
{\
	val = max;\
}\

/*================ CHASSIS MOVE SPEED RATIO ==================*/
#define CHASSIS_RC_MOVE_RATIO_X 1.0f 
#define CHASSIS_RC_MOVE_RATIO_Y 1.0f 
#define CHASSIS_PC_MOVE_RATIO_X 1.0f 
#define CHASSIS_PC_MOVE_RATIO_Y 1.0f 


/*================ GIMBAL MOVE SPEED RATIO ==================*/
#define GIMBAL_RC_MOVE_RATIO_PIT 1.0f
#define GIMBAL_RC_MOVE_RATIO_YAW 1.5f

#define GIMBAL_PC_MOVE_RATIO_PIT 1.0f
#define GIMBAL_PC_MOVE_RATIO_YAW 1.0f
/*================ GIMBAL MOVE SPEED RATIO ==================*/

/*================ GIMBAL SHOT PART PARAM ==================*/
#define trigger_MOTOR_SPEED 2000 //biger = faster
#define SHOT_FRIC_WHEEL_SPEED 1580 //biger = faster, max = 2000
/*================ GIMBAL SHOT PART PARAM ==================*/

/*================ Calibrate Variable Part ==================*/
/**
	@chinese 	UTF-8 encoding
	校准云台，使用jlink debug模式，将云台扶正到中间 
	将gAppParam.Gimbal.NeedCali 设为1即可
	*/
/*================ Calibrate Variable Part ==================*/

/*================ Error Detect Part ==================*/
/**
	when error occurs Red LED will flash. otherwise Green LED lights.
	in debug mode, see variable [gRxErr] gRxErr.str = "xxxx" 
											eg: gyro lose => gRxErr.str"gyro"
	*/
/*================ Error Detect Part ==================*/

#endif
