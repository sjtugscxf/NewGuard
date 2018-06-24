/**
  ******************************************************************************
  * File Name          : ShootTask.c
  * Description        : �������Ħ���ֿ��ƣ�
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
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;		//Ħ����б��
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   	//�����ٶ�б��
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;
uint16_t remoteShootDelay = 500;
static uint32_t RotateCNT = 0;	//������������
static uint16_t CNT_1s = 75;		//���ڱ���������ģʽ������������8�������ܼ������
static uint16_t CNT_250ms = 18;	

//�ֱ�ģʽ������ƺ���	
void SetFrictionWheelSpeed(uint16_t x)
{
	//__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_1, x);//������Ŀ����
	//__HAL_TIM_SET_COMPARE(&FRICTION_TIM, TIM_CHANNEL_2, x);
}
	
//ң��������Ħ����
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
				SetFrictionWheelSpeed(1000);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
			}
			else
			{
				SetFrictionWheelSpeed(1000 + (FRICTION_WHEEL_MAX_DUTY-1000)*frictionRamp.Calc(&frictionRamp)); 
				if(frictionRamp.IsOverflow(&frictionRamp))
				{
					FrictionWheelState = FRICTION_WHEEL_ON; 	
				}
				
			}
		}break;
		case FRICTION_WHEEL_ON:
		{
			if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   
			{
				FrictionWheelState = FRICTION_WHEEL_OFF;				  
				SetFrictionWheelSpeed(1000); 
				frictionRamp.ResetCounter(&frictionRamp);
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