/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       gimbal_task.h
	* @brief      basic gimbal control 
	* @update	        
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo      basic gimbal control
  * V1.0.1      Jun-05-2017   Richard.luo      add pitch and yaw back center after
  *                                            gimbal modle change from relax to work.
  * @verbatim
	* yaw axis  : angle feedback is single axis gyro, palstance feedback is mpu6500 gyro
	* pitch axis: angle feedback is encoder, palstance feedback is mpu6500 gyro
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#include "mytype.h"
void gimbal_task(const void* argu);

typedef enum {

    GIMBAL_INIT = 0,
    GIMBAL_RELAX,
		GIMBAL_AUTO_SHOOT,
    GIMBAL_CLOSE_LOOP_ZGYRO,

} eGimbalMode;

typedef struct
{
    eGimbalMode ctrl_mode; //yaw
    eGimbalMode last_mode; //

    float zgyro_target;
    float zgyro_angle;
    float zgyro_offset;

} gimbal_yaw_t;

extern gimbal_yaw_t gYaw;
extern s16  yaw_relative_pos;

#endif

