/**
  ******************************************************************************
  * File Name          : ShootTask.c
  * Description        : 射击任务（摩擦轮控制）
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
FrictionWheelState_e FrictionWheelState;
Shoot_State_e ShootState;
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;		//摩擦轮斜坡
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//键盘速度斜坡
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
uint16_t remoteShootDelay = 500;
static uint32_t RotateCNT = 0;	//长按连发计数
static uint16_t CNT_1s = 75;		//用于避免四连发模式下两秒内连射8发过于密集的情况
static uint16_t CNT_250ms = 18;	

//手柄模式射击控制函数	
void SetFrictionWheelSpeed(float x)
{
	friclSpeedTarget = x;
	fricrSpeedTarget = -x;
}
	
//遥控器开启摩擦轮
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val) 
{
	switch(FrictionWheelState)
	{
		case FRICTION_WHEEL_OFF:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)   
			{
				ShootState = NOSHOOTING;
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_START_TURNNING;	 
			}				 		
		}break;
		case FRICTION_WHEEL_START_TURNNING:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				ShootState = NOSHOOTING;
				SetFrictionWheelSpeed(0);
				FrictionWheelState = FRICTION_WHEEL_OFF;
			}
			else
			{
				SetFrictionWheelSpeed(400); 
				FrictionWheelState = FRICTION_WHEEL_ON; 				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				FrictionWheelState = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(0); 
				ShootState = NOSHOOTING;
			}
			else if(sw->switch_value_raw == 2)
			{
				ShootState = SHOOTING;
			}
			else
			{
				ShootState = NOSHOOTING;
			}					 
		} break;				
	}
}