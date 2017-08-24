/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       chassis_task.c
	* @brief      provide basic chassis control, use chassis follow gimbal model
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo      basic chassis control    
  * @verbatim
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
	
#ifndef __CHASSIS_TASK_H__
#define __CHASSIS_TASK_H__

#include "mytype.h"


typedef enum {
	  CHASSIS_AUTO = 0, 
	
    CHASSIS_STOP, // stall
    CHASSIS_RELAX, // relax
    CHASSIS_OPEN_LOOP,
    CHASSIS_CLOSE_GYRO_LOOP,
    CHASSIS_FOLLOW_GIMBAL,

} eChassisMode;

typedef struct
{
    float           vx; // forward/back
    float           vy; // left/right
    float           vw; // 
    float           kb_vx;
    float           kb_vy;
    float           mouse_vw;
    eChassisMode    mode;
    eChassisMode    last_mode;
    float           target_angle; //
    float           angle_from_gyro;
		float           palstance_from_gyro;
    float           last_angle;
    int             is_snipe_mode; //gimbal chassis separate
    int16_t         wheel_speed[4];
} chassis_t; //chassis status

void mecanumCalc(float vx, float vy, float vw, const int each_max_spd,
                  s16 speed[]);
void reset_zgyro(void);
void chassis_pid_init(void);
void chassis_task(void const*);

extern chassis_t chassis;
#endif
