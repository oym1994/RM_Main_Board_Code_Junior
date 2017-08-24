/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       chassis_task.c
	* @brief      provide basic chassis control, use chassis follow gimbal model
	* @update	  
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Apr-30-2017   Richard.luo      basic chassis control    
  * @verbatim
	*  can use dji remoter or mouse and keyboard control chassis move
	********************************(C) COPYRIGHT 2017 DJI************************
	*/
#include "chassis_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "can.h"
#include "cmsis_os.h"
#include "error_task.h"
#include "gimbal_task.h"
#include "judge_sys.h"
#include "kb.h"
#include "pid.h"
#include "mpu.h"

#include "sys.h"
#include <math.h>
#define MyAbs(x) ((x > 0) ? (x) : (-x))
#define MAX_WHEEL_SPEED 750
#define MAX_CHASSIS_VX_SPEED 400
#define MAX_CHASSIS_VY_SPEED 400
#define MAX_CHASSIS_VR_SPEED 350

chassis_t chassis; 
s16 current_3510[4];

/* for debuge */
int wheel_s_f, wheel_s_r;

/**
  * @brief     reset single axis gyroscope 
  * @attention gyro reset at least wait 2s  
  */
void reset_zgyro(void)
{
    while (ZGYRO_CAN.State == HAL_CAN_STATE_BUSY_TX);
    ZGYRO_CAN.pTxMsg->StdId   = CAN_ZGYRO_RST_ID;
    ZGYRO_CAN.pTxMsg->IDE     = CAN_ID_STD;
    ZGYRO_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    ZGYRO_CAN.pTxMsg->DLC     = 0x08;
    ZGYRO_CAN.pTxMsg->Data[0] = 0;
    ZGYRO_CAN.pTxMsg->Data[1] = 1;
    ZGYRO_CAN.pTxMsg->Data[2] = 2;
    ZGYRO_CAN.pTxMsg->Data[3] = 3;
    ZGYRO_CAN.pTxMsg->Data[4] = 4;
    ZGYRO_CAN.pTxMsg->Data[5] = 5;
    ZGYRO_CAN.pTxMsg->Data[6] = 6;
    ZGYRO_CAN.pTxMsg->Data[7] = 7;
    HAL_CAN_Transmit(&ZGYRO_CAN, 1000);       
		
}

/**
	* @brief mecanum calculation function
  * @param input : vx vy vw(+ cw, - ccw)
  *        output: 4 wheel speed
	* @note  1=FR 2=FL 3=BL 4=BR
	* @map 	 2	%++++++%	1
					 			++++
								++++
					 3	%++++++%	4    ↑=+Vy  →=+Vx
  */
void mecanum_calc(float vx, float vy, float vw, const int each_max_spd, s16 speed[])
{
	s16   buf[4];
	int   i;
	float max = 0, rate;

	VAL_LIMIT(vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
  VAL_LIMIT(vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);
  VAL_LIMIT(vw, -MAX_CHASSIS_VR_SPEED, MAX_CHASSIS_VR_SPEED);

	buf[0] = (+vx - vy + vw);
	buf[1] = (+vx + vy + vw);
	buf[2] = (-vx + vy + vw);	
	buf[3] = (-vx - vy + vw);

	//find max item
	for (i = 0; i < 4; i++)
	{
			if (MyAbs(buf[i]) > max)
					max = MyAbs(buf[i]);
	}
	//equal proportion
	if (max > each_max_spd)
	{
			rate = each_max_spd / max;
			for (i = 0; i < 4; i++)
					buf[i] *= rate;
	}
	memcpy(speed, buf, sizeof(s16) * 4);
}
/**
  * @brief     send 4 calculated current to motor
  * @param     3510 motor ESC id
  * @retval    none
  */
void set_cm_current(CAN_HandleTypeDef *hcan, s16 iq1, s16 iq2, s16 iq3, s16 iq4)
{

    hcan->pTxMsg->StdId   = 0x200;
    hcan->pTxMsg->IDE     = CAN_ID_STD;
    hcan->pTxMsg->RTR     = CAN_RTR_DATA;
    hcan->pTxMsg->DLC     = 0x08;
    hcan->pTxMsg->Data[0] = iq1 >> 8;
    hcan->pTxMsg->Data[1] = iq1;
    hcan->pTxMsg->Data[2] = iq2 >> 8;
    hcan->pTxMsg->Data[3] = iq2;
    hcan->pTxMsg->Data[4] = iq3 >> 8;
    hcan->pTxMsg->Data[5] = iq3;
    hcan->pTxMsg->Data[6] = iq4 >> 8;
    hcan->pTxMsg->Data[7] = iq4;
    HAL_CAN_Transmit(hcan, 1000);
}




void get_chassis_mode_set_ref(RC_Type *rc)
{
	chassis.last_mode = chassis.mode;

	switch (rc->sw1)
  {
    case RC_UP:
      chassis.mode = CHASSIS_AUTO;  //senior students no use
    break;

    case RC_MI:
			//chassis.mode = CHASSIS_CLOSE_GYRO_LOOP;
      chassis.mode = CHASSIS_FOLLOW_GIMBAL; //
    break;

    case RC_DN:
		  chassis.mode = CHASSIS_OPEN_LOOP;  //senior students no use
    break;
		
    default:
		break;
  }
/*
	switch (chassis.mode)
	{
		case CHASSIS_AUTO:
		{		
			//control coordinate is based on rc, where +y is forward, +x is right, +w is clockwise
			//auto_vx, auto_vy or auto_wv is measured by mm/s and deg/s
		}break;
		
		case CHASSIS_OPEN_LOOP:
		{
			chassis.vy = rc->ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy;
			chassis.vx = rc->ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx;
			chassis.vw = rc->ch3 / 2 + rc->mouse.x * 10; 
		}break;
		
		case CHASSIS_FOLLOW_GIMBAL: 
		{
		  chassis.vy = rc->ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy;
			chassis.vx = rc->ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx;
		}break;
		
		case CHASSIS_CLOSE_GYRO_LOOP:
		{
			chassis.vy = rc->ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy;
			chassis.vx = rc->ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx;
			chassis.target_angle += -rc->ch3 * 0.001f;				
		}break;
		
		default :
		{
		}break;
	}
*/
}


/**
  * @brief     initialize chassis motor pid parameter
  * @usage     before chassis loop use this function
  */
void chassis_pid_param_init(void)
{
	for (int k = 0; k < 4; k++)
	{
		//max current = 20000
		PID_struct_init(&pid_spd[k], POSITION_PID, 10000, 1000, 4, 0.05f, 5.0f);
	}
	PID_struct_init(&pid_chassis_angle, POSITION_PID, 600, 80, 1.0f, 0.0f, 0.0f);
	pid_chassis_angle.max_err  = 60 * 22.75f; // err angle > 60 cut the output
	pid_chassis_angle.deadband = 35;
}

void chassis_task(void const* argu)
{
	int i = 0;
	
	chassis_pid_param_init();

	HAL_Delay(1000);
    
	while (1)
	{
		pc_kb_hook();

		get_chassis_mode_set_ref(&rc);

		
		switch (chassis.mode)
		{
			case CHASSIS_FOLLOW_GIMBAL:
			{
				chassis.vy = rc.ch2 * CHASSIS_RC_MOVE_RATIO_Y + km.vy * CHASSIS_PC_MOVE_RATIO_Y;
				chassis.vx = rc.ch1 * CHASSIS_RC_MOVE_RATIO_X + km.vx * CHASSIS_PC_MOVE_RATIO_X;		
			}break;
	
			default:
			{
				chassis.vy = 0;
				chassis.vx = 0;
			}break;
		}
		
		if (gYaw.ctrl_mode == GIMBAL_CLOSE_LOOP_ZGYRO)
		{
			chassis.vw = pid_calc(&pid_chassis_angle, yaw_relative_pos, 0); 
		}
		else
		{
			chassis.vw = 0;
		}

		mecanum_calc(chassis.vx, chassis.vy, chassis.vw, MAX_WHEEL_SPEED,
								 chassis.wheel_speed);
		for (i = 0; i < 4; i++)
		{
				current_3510[i] = pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm,
																	chassis.wheel_speed[i] * 10);
		}
		
		wheel_s_f = moto_chassis[0].speed_rpm;
		wheel_s_r = chassis.wheel_speed[0] * 10;

		if (chassis.mode == CHASSIS_RELAX || gRxErr.err_list[DbusTOE].err_exist)
		{
				memset(current_3510, 0, sizeof(current_3510));
				pid_spd[0].iout = 0;
				pid_spd[1].iout = 0;
				pid_spd[2].iout = 0;
				pid_spd[3].iout = 0;
		}

		set_cm_current(&CHASSIS_CAN, current_3510[0], current_3510[1], current_3510[2],
									 current_3510[3]);

		osDelay(10);
	}
}
