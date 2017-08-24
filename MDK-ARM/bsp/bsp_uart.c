/**
  ********************************(C) COPYRIGHT 2017 DJI************************
  * @file       bsp_uart.c
	* @brief      uart receive data from DBus/bt/judge_system/manifold etc.
	* @update
  * @note       use DMA receive, but donot trigger DMA interrupt
	*             handle received data in usart idle interrupt handle function
	*	@history
	* Version     Date          Author           Modification
  * V1.0.0      Jun-01-2017   Richard.luo      remove some useless module
  * @verbatim
	*		idle interrupt --> handle data --> clear it flag --> initialize DMA again
	*
	********************************(C) COPYRIGHT 2017 DJI************************
	*/

#include "bsp_uart.h"
#include "error_task.h"
#include "judge_sys.h"
#include "mytype.h"
#include "pid.h"
#include "sys.h"
#include "usart.h"

#define MAX_DMA_COUNT 100
#define DBUS_RX_MAX_BUFLEN 20
#define AUTOP_SIZE 17   //sizeof(tReceXToneTData)
#define SEND_SIZE 28    //sizeof(tSendTXoneData)

//#define ARMAPI extern "C" // add this before hal callback

/* remote control data */
RC_Type  rc;
u8       dbus_buff[DBUS_RX_MAX_BUFLEN];

/* vision_buff  useless */
tSendTXoneData SendData;
tReceTXoneData ReceData;
uint8_t vision_buff[AUTOP_SIZE+1];
uint8_t auto_send[SEND_SIZE+1];

/**
  * @brief   enable global uart it and do not use DMA transfer done it
  * @param   uart IRQHandler id, receive buff, buff size
  * @retval  set success or fail
  */
static int UART_Receive_DMA_No_IT(UART_HandleTypeDef* huart, u8* pData, u32 Size)
{
	uint32_t tmp1 = 0;

	tmp1 = huart->RxState;
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
				return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode  = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR,
									(uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
		in the UART CR3 register */
		huart->Instance->CR3 |= USART_CR3_DMAR;

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

/**
  * @brief   initialize uart device 
  * @usage   after MX_USARTx_UART_Init() use these function
  */
void dbus_init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&DBUS_HUART);
    __HAL_UART_ENABLE_IT(&DBUS_HUART, UART_IT_IDLE);
    // clear idle it flag & open idle it
    UART_Receive_DMA_No_IT(&DBUS_HUART, dbus_buff, DBUS_RX_MAX_BUFLEN);
}
void judge_sys_init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&JUDGE_HUART);
    __HAL_UART_ENABLE_IT(&JUDGE_HUART, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&JUDGE_HUART, judge_buf, FRAME_BUFLEN);
}
void manifold_uart_init()
{
    __HAL_UART_CLEAR_IDLEFLAG(&CV_HUART);
    __HAL_UART_ENABLE_IT(&CV_HUART, UART_IT_IDLE);
    UART_Receive_DMA_No_IT(&CV_HUART, vision_buff, AUTOP_SIZE);
}


/**
  * @brief   clear idle it flag after uart receive a frame data
  * @param   uart IRQHandler id
  * @retval  none
  * @usage   call in MyUartFrameIRQHandler() function
  */
void uart_reset_idle_rx_callback(UART_HandleTypeDef* huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		// clear idle it flag
		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx); 
		//according uart clear corresponding DMA flag

		__HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, MAX_DMA_COUNT);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}
void Uart_Callback_RC_Handle(RC_Type* rc, uint8_t* buff)
{
    if (buff[0] == 0 && buff[1] == 0 && buff[2] == 0 && buff[3] == 0 && buff[4] == 0 && buff[5] == 0)
        return;

    rc->ch1 = (buff[0] | buff[1] << 8) & 0x07FF;
    rc->ch1 -= 1024;
    rc->ch2 = (buff[1] >> 3 | buff[2] << 5) & 0x07FF;
    rc->ch2 -= 1024;
    rc->ch3 = (buff[2] >> 6 | buff[3] << 2 | buff[4] << 10) & 0x07FF;
    rc->ch3 -= 1024;
    rc->ch4 = (buff[4] >> 1 | buff[5] << 7) & 0x07FF;
    rc->ch4 -= 1024;

		if(rc->ch1 <= 7 && rc->ch1 >= -7)
			rc->ch1 = 0;
		if(rc->ch2 <= 4 && rc->ch2 >= -4)
			rc->ch2 = 0;
		if(rc->ch3 <= 4 && rc->ch3 >= -4)
			rc->ch3 = 0;		
		if(rc->ch4 <= 4 && rc->ch4 >= -4)
			rc->ch4 = 0;

    rc->sw1 = ((buff[5] >> 4) & 0x000C) >> 2;
    rc->sw2 = (buff[5] >> 4) & 0x0003;

    rc->mouse.x = buff[6] | (buff[7] << 8); // x axis
    rc->mouse.y = buff[8] | (buff[9] << 8);
    rc->mouse.z = buff[10] | (buff[11] << 8);

    rc->mouse.l = buff[12]; //
    rc->mouse.r = buff[13];

    rc->kb.key_code = buff[14] | buff[15] << 8; // key borad code
}

/**
  * @brief   callback this function when uart interrupt 
  * @param   uart IRQHandler id
  * @retval  none
  * @usage   call in uart handler function USARTx_IRQHandler()
  */
void MyUartFrameIRQHandler(UART_HandleTypeDef *huart)
{
	if (huart == &DBUS_HUART)
	{
		Uart_Callback_RC_Handle(&rc, dbus_buff);
		err_detector_hook(DbusTOE);
	}
	else if (huart == &JUDGE_HUART)
	{
		judgementDataHandler();
	}
	else if (huart == &CV_HUART)
	{
		memcpy(&ReceData, vision_buff, sizeof(tReceTXoneData));
		//USAGE:
		//add your code if you want communicating with miniPC(TXone/manifold etc...)
		//get data after CRC or frame head and tail verification

	}
	uart_reset_idle_rx_callback(huart);
}



/**
  * @brief   send data to your manifold/bluetooth/tx1/pc etc... 
  * @param   uart id
  * @retval  none
  * @usage   call this function where your code need send data
  */
void uart_send_data(UART_HandleTypeDef *huart)
{
//add your code if you want communicating with miniPC(TXone/manifold etc...)
//better add CRC or frame head and tail verification
//example :
//SendData.xxx = xxx;
	
	memcpy(auto_send, &SendData, sizeof(tSendTXoneData));
	HAL_UART_Transmit(huart, auto_send, sizeof(tSendTXoneData), 30);
}




