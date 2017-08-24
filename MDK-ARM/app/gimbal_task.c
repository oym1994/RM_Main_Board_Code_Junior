/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       gimbal_task.c
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

#include "gimbal_task.h"
#include "bsp_can.h"
#include "bsp_uart.h"
#include "calibrate.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "error_task.h"
#include "judge_sys.h"
#include "kb.h"
#include "mpu.h"
#include "mytype.h"
#include "pid.h"
#include "stm32f4xx_hal.h"
#include "sys.h"
#include <math.h>

/* for debuge */
int pit_p_r, pit_p_f, pit_s_r, pit_s_f;
int yaw_p_r, yaw_p_f, yaw_s_r, yaw_s_f;

/* if college's infantry, enable this define */
#define COLLEGE

gimbal_yaw_t gYaw;

/* gimbal pid parameter */
float yaw_angle_ref     = 0;
float pit_angle_ref     = 0; 
float yaw_angle_fdb     = 0;
float pit_angle_fdb     = 0; 
		
/* read from flash */
int   pit_center_offset = 0; 
int   yaw_center_offset = 0;

/* gimbal relative position param */
s16    pit_relative_pos;
s16    yaw_relative_pos;    //unit: encoder
float  pit_relative_angle;
float  yaw_relative_angle;  //unit: degree

/* shoot task relevant param */
u32    continue_shoot_time;
u8     c_shoot_cmd = 0;
int    last_sw1;
int    fric_wheel_run = 0; //run or not
int    trigger_spd_ref;
int    trigger_pos_ref;
int    trigger_dir = -1;
u8     shoot_cmd = 0;

/* imu relevant param */
float  imu_tmp;
float  imu_tmp_ref = 40; //keep imu in 40 degree

/**
  * @brief     get relative position angle to center
  * @param[in] raw_ecd: gimbal motor encoder raw angle
	* @param[in] center_offset: read GimbalCaliData from chip flash
	* @retval    relative angle, unit is degree.
  * @attention you should read center offset data in chip flash, 
	*            as: GimbalCaliData.GimbalPit/Yaw Offset
  */
s16 get_relative_pos(s16 raw_ecd, s16 center_offset)
{
    s16 tmp = 0;
    if (center_offset >= 4096)
    {
        if (raw_ecd > center_offset - 4096)
            tmp = raw_ecd - center_offset;
        else
            tmp = raw_ecd + 8192 - center_offset;
    }
    else
    {
        if (raw_ecd > center_offset + 4096)
            tmp = raw_ecd - 8192 - center_offset;
        else
            tmp = raw_ecd - center_offset;
    }
    return tmp;
}

/**
  * @brief     send current which pid calculate to esc. message to calibrate 6025 gimbal motor esc
  * @param     current value corresponding motor(yaw/pitch/trigger)
  */
void can_send_gimbal_iq(s16 yaw_iq, s16 pit_iq, s16 trigger_iq)
{
    GIMBAL_CAN.pTxMsg->StdId   = 0x1ff;
    GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
    GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    GIMBAL_CAN.pTxMsg->DLC     = 8;
    GIMBAL_CAN.pTxMsg->Data[0] = yaw_iq >> 8;
    GIMBAL_CAN.pTxMsg->Data[1] = yaw_iq;
    GIMBAL_CAN.pTxMsg->Data[2] = pit_iq >> 8;
    GIMBAL_CAN.pTxMsg->Data[3] = pit_iq;
    GIMBAL_CAN.pTxMsg->Data[4] = trigger_iq >> 8;
    GIMBAL_CAN.pTxMsg->Data[5] = trigger_iq;
    GIMBAL_CAN.pTxMsg->Data[6] = 0;
    GIMBAL_CAN.pTxMsg->Data[7] = 0;
    HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
}


/**
  * @brief     send particular message to calibrate 6025 gimbal motor esc
  * @usage     only need calibrate motor when replace a new ESC
  */
void can_cali_6025(void)
{

    GIMBAL_CAN.pTxMsg->StdId   = 0x3f0;
    GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
    GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    GIMBAL_CAN.pTxMsg->DLC     = 8;
    GIMBAL_CAN.pTxMsg->Data[0] = 'c';
    GIMBAL_CAN.pTxMsg->Data[1] = 0;
    GIMBAL_CAN.pTxMsg->Data[2] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[3] = 0;
    GIMBAL_CAN.pTxMsg->Data[4] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[5] = 0;
    GIMBAL_CAN.pTxMsg->Data[6] = 0;
    GIMBAL_CAN.pTxMsg->Data[7] = 0;
    HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
}
/**
  * @brief     send particular message to calibrate 6623 gimbal motor esc
  * @usage     only need calibrate motor when replace a new ESC
  */
void can_cali_6623(void)
{

    GIMBAL_CAN.pTxMsg->StdId   = 0x1ff;
    GIMBAL_CAN.pTxMsg->IDE     = CAN_ID_STD;
    GIMBAL_CAN.pTxMsg->RTR     = CAN_RTR_DATA;
    GIMBAL_CAN.pTxMsg->DLC     = 8;
    GIMBAL_CAN.pTxMsg->Data[0] = 0;
    GIMBAL_CAN.pTxMsg->Data[1] = 0;
    GIMBAL_CAN.pTxMsg->Data[2] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[3] = 0;
    GIMBAL_CAN.pTxMsg->Data[4] = 0 >> 8;
    GIMBAL_CAN.pTxMsg->Data[5] = 0;
    GIMBAL_CAN.pTxMsg->Data[6] = 4;
    GIMBAL_CAN.pTxMsg->Data[7] = 0;
    HAL_CAN_Transmit(&GIMBAL_CAN, 1000);
}



/**
  * @brief     initialize gimbal pid parameter, such as pitch/yaw/trigger motor, 
  *            imu temperature
  * @attention before gimbal control loop in gimbal_task() function
  */
void gimbal_pid_init(void)
{
	PID_struct_init(&pid_pit, POSITION_PID, 4000, 1000,
									200.0f, 0.5, 0); //
	PID_struct_init(&pid_pit_speed, POSITION_PID, 4000, 1000,
									20.0f, 0.2, 0.8f);

	PID_struct_init(&pid_yaw, POSITION_PID, 9000, 400,
									180, 0.4, 150); //
	PID_struct_init(&pid_yaw_speed, POSITION_PID, 8000, 800,
									20, 0.3, 20);
	
	PID_struct_init(&pid_trigger, POSITION_PID, 10000, 2000, //position
									15.0f, 0, 10);
	PID_struct_init(&pid_trigger_speed, POSITION_PID, 7000, 4000, //vw
									1.5f, 0.3f, 5);

	PID_struct_init(&pid_imu_tmp, POSITION_PID, 1000, 300,
									180, 0.1f, 0);
}


void shoot_task(void)
{
	if (fric_wheel_run)
	{
		if ((rc.sw1 == RC_DN && last_sw1 == RC_MI) || left_key)
		{
		  continue_shoot_time = HAL_GetTick();
		  shoot_cmd = 1;
		}
		
	  if (shoot_cmd)
		{
			trigger_pos_ref = moto_trigger.total_ecd;
      trigger_pos_ref += 130922 * trigger_dir;
		  shoot_cmd = 0;
		}
		
		if (rc.sw1 == RC_DN)
		{
			if (HAL_GetTick() - continue_shoot_time >= 800)
			{
				c_shoot_cmd = 1;
			}
		}
	}
	
	if (rc.sw1 == RC_MI)
	{
		if (last_sw1 == RC_DN && c_shoot_cmd)
			trigger_pos_ref = moto_trigger.total_ecd;
		
		c_shoot_cmd = 0;
		shoot_cmd = 0;
	}
	
  /* turn off/on friction wheel */
	if (rc.kb.bit.Z && rc.sw2 == RC_UP)
	{
		fric_wheel_run = 1;
	}
	if ((rc.kb.bit.Z && rc.kb.bit.SHIFT) || rc.sw2 != RC_UP)
	{
		fric_wheel_run = 0;
	}
	if (last_sw1 == RC_MI && rc.sw1 == RC_UP && rc.sw2 == RC_UP)
	{
			fric_wheel_run = !fric_wheel_run;
	}
	last_sw1 = rc.sw1;
	
	
	if (fric_wheel_run && rc.sw2 == RC_UP && !gRxErr.err_list[DbusTOE].err_exist)
	{
		TIM12->CCR1 = TIM12->CCR2 = SHOT_FRIC_WHEEL_SPEED;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
	}
	else
	{
		TIM12->CCR1 = TIM12->CCR2 = 1000;
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
	}
}
/**
  * @brief     gimbal control task
  * @attention the yaw angle feedback is single axis gyro, palstance feedback is mpu6500 gyro
  *            so maybe happen positive feedback, 
  *            if this case, you should modify their parameter sign.
  */
void gimbal_task(const void* argu)
{
	gimbal_pid_init();
	/* for angular rate measure */
	mpu_device_init();
	/* read gimbal offset from gAppParam */
	if (gAppParam.GimbalCaliData.isAlreadyCalied == CALIED_FLAG)
	{
		yaw_center_offset = gAppParam.GimbalCaliData.GimbalYawOffset;
		pit_center_offset = gAppParam.GimbalCaliData.GimbalPitOffset;
	}
	else
	{
		//while (1);
	}
	/* if replace new motor esc, need calibrate */
	//can_cali_6025();
	//can_cali_6623();
	
	while (1)
	{
		/* keep imu temperature in 40 degree */
		imu_tmp = 21 + mpu_data.temp / 333.87f;
		pid_calc(&pid_imu_tmp, imu_tmp, imu_tmp_ref);
		TIM3->CCR2 = pid_imu_tmp.pos_out;

		/* get Z axis palstance, for gimbal speed loop feedback */
		mpu_get_data();
		gimbalCaliHook();
	
		gYaw.last_mode = gYaw.ctrl_mode;
		switch (rc.sw2)
		{
			case (RC_UP):
			{
				if (gYaw.ctrl_mode != GIMBAL_INIT)
					gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
						
				/* gimbal back to center, can modify pid parameter to make gimbal slow in this model */
				if (gYaw.last_mode == GIMBAL_RELAX && gYaw.ctrl_mode != GIMBAL_RELAX)
					gYaw.ctrl_mode = GIMBAL_INIT;
			}
			break;
			default:
				gYaw.ctrl_mode = GIMBAL_RELAX;
			break;
		}

		shoot_task();

		//transform absolute ecd range [0,8192] to relative range
		yaw_relative_pos = -get_relative_pos(moto_yaw.angle, yaw_center_offset);
		pit_relative_pos = get_relative_pos(moto_pit.angle, pit_center_offset);			
		
		pit_relative_angle = pit_relative_pos/22.75f;
		yaw_relative_angle = yaw_relative_pos/22.75f;
	
		
		switch (gYaw.ctrl_mode)
		{
			case GIMBAL_INIT:
			{
				/* lift gimbal pitch */
				pit_angle_fdb = pit_relative_angle;
				pit_angle_ref = 0;
				/* keep yaw unmove this time */
				yaw_angle_fdb = 0;
				yaw_angle_ref = 0;
				
				if(pit_angle_fdb >= -10.0f && pit_angle_fdb <= 3.0f)
				{
					/* yaw back center after pitch arrive */
					yaw_angle_fdb = yaw_relative_angle;
				  yaw_angle_ref = 0;
					
					if (yaw_angle_fdb >= -1.0f && yaw_angle_fdb <= 1.0f)
					{
						/* yaw arrive and switch gimbal state */
						gYaw.ctrl_mode = GIMBAL_CLOSE_LOOP_ZGYRO;
						
						gYaw.zgyro_offset = yaw_zgyro_angle;
						pit_angle_ref = 0;
						yaw_angle_ref = 0;
					}
				}
			}break;
			
			case GIMBAL_CLOSE_LOOP_ZGYRO:
			{
				pit_angle_fdb = pit_relative_angle;
				pit_angle_ref += rc.ch4 * 0.003
                       - rc.mouse.y * 0.01f;

				yaw_angle_fdb = yaw_zgyro_angle - gYaw.zgyro_offset;
				yaw_angle_ref += -rc.ch3 * 0.0015f
											 - rc.mouse.x * 0.01f;  
			}break;
			
			default:
				break;
		}
		
		pid_calc(&pid_yaw, yaw_angle_fdb, yaw_angle_ref);
		pid_calc(&pid_yaw_speed, mpu_data.gz / 10.0f, pid_yaw.pos_out / 10.0f);

		/* pitch axis limited angle */
#if defined(COLLEGE)
		VAL_LIMIT(pit_angle_ref, -10, 25);
#else
		VAL_LIMIT(pit_angle_ref, -20, 25);
#endif
		pid_calc(&pid_pit, pit_angle_fdb, pit_angle_ref);
		pid_calc(&pid_pit_speed, mpu_data.gx / 20.0f, pid_pit.pos_out / 10.0f); //

		
		
		pid_calc(&pid_trigger, moto_trigger.total_ecd / 100, trigger_pos_ref / 100);
		if (c_shoot_cmd)
			trigger_spd_ref = -4000;
		else
			trigger_spd_ref = pid_trigger.pos_out;
		pid_calc(&pid_trigger_speed, moto_trigger.speed_rpm, trigger_spd_ref);
		
		/* for debuge */
		pit_p_f = pit_angle_fdb * 1000;
		pit_p_r = pit_angle_ref * 1000;
		yaw_p_f = yaw_angle_fdb * 1000;
		yaw_p_r = yaw_angle_ref * 1000;
		yaw_s_f = mpu_data.gz   * 100;
		yaw_s_r = pid_yaw.pos_out * 100;
		

		/* final output, for safe protect purpose */
		if (gAppParam.GimbalCaliData.isAlreadyCalied == CALIED_FLAG
				&& gYaw.ctrl_mode != GIMBAL_RELAX
				&& !gRxErr.err_list[DbusTOE].err_exist
				&& !gRxErr.err_list[GimbalYawTOE].err_exist
				&& !gRxErr.err_list[GimbalPitTOE].err_exist)
		{
			can_send_gimbal_iq(pid_yaw_speed.pos_out, -pid_pit_speed.pos_out, pid_trigger_speed.pos_out);
		}
		else
		{
			pid_trigger.iout = 0;
			can_send_gimbal_iq(0, 0, 0); //relax state
		}
		
		osDelay(5);
	}
}
