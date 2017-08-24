#include "kb.h"
#include "bsp_uart.h"
#include "chassis_task.h"
#include "cmsis_os.h"
#include "mytype.h"
#include "sys.h"

km_control_t km;
u16  key_code          = 0;
vu8  kb_move_mode      = 0;
u16  max_kb_move_speed = 400;

u8   left_key, right_key;
u8   left_key_sta  = 0;
u8   right_key_sta = 0;

enum
{
  KEY_IDLE = 0,          // no key action, idle sta
  KEY_DOWN_WAIT_CONFIRM, // maybe in key shake sta, need to confirm according to next time.
  KEY_DOWN_WAIT_RELEASE, // after key release, a full key press action is done.
} KeyStateTypeDef;

// used for avoid mouse press shake
u8 key_fsm(u8* psta, u8 condition)
{
  u8 ret = 0; 
  switch (*psta)
  {
    case KEY_IDLE:
      if (condition)
        *psta = KEY_DOWN_WAIT_CONFIRM;
      break;

    case KEY_DOWN_WAIT_CONFIRM:
      if (condition)
      {
        *psta = KEY_DOWN_WAIT_RELEASE;
        ret   = 1; // triggered when key is not release yet.
      }
      else
        *psta = KEY_IDLE;
      break;

    case KEY_DOWN_WAIT_RELEASE:
      if (!condition)
        *psta = KEY_IDLE;
      break;
  }
  return ret;
}

void pc_kb_hook(void)
{
//	if(rc.kb.bit.SHIFT)
//		kb_move_mode = ShiftQuicklyMode;
//	else if(rc.kb.bit.CTRL)
//		kb_move_mode = CtrlSlowlyMode;
//	else
//		kb_move_mode = NormalMode;

	//add ramp
  if (rc.kb.bit.W)
    km.vy += 5;
  else if (rc.kb.bit.S)
    km.vy -= 5;
  else
    km.vy = 0;

  if (rc.kb.bit.A)
    km.vx += -5;
  else if (rc.kb.bit.D)
    km.vx += 5;
  else
    km.vx = 0;

	VAL_LIMIT(km.vx, -max_kb_move_speed, max_kb_move_speed);
	VAL_LIMIT(km.vy, -max_kb_move_speed, max_kb_move_speed);
	
	left_key = key_fsm(&left_key_sta, rc.mouse.l);
	right_key = key_fsm(&right_key_sta, rc.mouse.r);
}
