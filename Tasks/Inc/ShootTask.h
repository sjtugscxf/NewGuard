/**
  ******************************************************************************
  * File Name          : ShootTask.h
  * Description        : Éä»÷ÈÎÎñ£¨Ä¦²ÁÂÖ¿ØÖÆ£©
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

#define FRICTION3510

#define FRICTION_RAMP_TICK_COUNT				 100

#ifdef FRICTION3510
#define FRICTION_WHEEL_MAX_DUTY         150
#define FRICTION_WHEEL_ZERO         0
#else 
#define FRICTION_WHEEL_MAX_DUTY         1350
#define FRICTION_WHEEL_ZERO         1000
#endif

#define FRICTION_TIM  htim5

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

#ifdef FRICTION3510
void SetFrictionWheelSpeed(float x);
#else 
void SetFrictionWheelSpeed(uint16_t x);
#endif

extern RampGen_t frictionRamp;
						
#endif /* __SHOOTTASK_H */