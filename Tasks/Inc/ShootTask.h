/**
  ******************************************************************************
  * File Name          : ShootTask.h
  * Description        : �������Ħ���ֿ��ƣ�
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __SHOOTTASK_H
#define __SHOOTTASK_H

#include "includes.h"

#define FRICTION_RAMP_TICK_COUNT				100
#define FRICTION_WHEEL_MAX_DUTY         1350
#define FRICTION_TIM  htim12

typedef __packed enum
{
	FRICTION_WHEEL_OFF = 0,
	FRICTION_WHEEL_START_TURNNING = 1,
	FRICTION_WHEEL_ON = 2,
}FrictionWheelState_e;

typedef __packed enum
{
	NOSHOOTING = 0,
	SHOOTING = 1,
}Shoot_State_e;

void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val);
void InitUserTimer(void);
void SetFrictionWheelSpeed(float x);
extern RampGen_t frictionRamp;
						
#endif /* __SHOOTTASK_H */