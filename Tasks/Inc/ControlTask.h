/**
  ******************************************************************************
  * File Name          : ControlTask.h
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#ifndef __CONTROLTASK_H
#define __CONTROLTASK_H

#include "includes.h"

#define AUTO_ATTACK_YAW_KP        1.5f
#define AUTO_ATTACK_YAW_KD        0.0f 
#define AUTO_ATTACK_PITCH_KP      0.0007f
#define AUTO_ATTACK_PITCH_KD      0.0f 
//#define YAW_OFFSET         320u  
//#define PITCH_OFFSET       210u  

#define ODOMETRY_FACT      0.01f 
#define ODOMETRY_UPMAX1    180000.0f 
#define ODOMETRY_DOWNMAX1  -180000.0f 
#define ODOMETRY_SPEED1    300.0f 
#define ODOMETRY_UPMAX2    70000.0f 
#define ODOMETRY_DOWNMAX2  -70000.0f 
#define ODOMETRY_SPEED2    200.0f 
#define ODOMETRY_SPEED3    40.0f 

#define PITCHANGLETARGETMAX1    68.0f 
#define PITCHANGLETARGETMAX2    68.0f 
#define PITCHANGLETARGETMIN1     0.0f 
#define PITCHANGLETARGETMIN2     0.0f 

#define YAW_DEFEND_SPEED       200.0f
#define PITCH_DEFEND_SPEED       0.08f

#define BULLET_SPEED       1200.0f  //bullet_ref = 400; Ã»ÓÐÅö×²¿ª¹ØµÄÊ±ºò
#define STOPHEAT       300u

#define CHASSIS_SPEED_ATTENUATION   (1.30f)
#define CHASSIS_MOTOR_ROTATE_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	1.4f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	4900,\
	1000,\
	1500,\
	0,\
	5000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define CHASSIS_MOTOR_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	6.5f,\
	0.0f,\
	1.0f,\
	0,\
	0,\
	0,\
	4900,\
	3500,\
	1500,\
	0,\
	4950,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define BULLET_POSITION_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	8.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	10000,\
	10000,\
	10000,\
	0,\
	10000,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

#define BULLET_SPEED_PID_DEFAULT \
{\
	0,\
	0,\
	{0,0},\
	2.0f,\
	0.0f,\
	0.0f,\
	0,\
	0,\
	0,\
	10000,\
	10000,\
	10000,\
	0,\
	4900,\
	0,\
	0,\
	0,\
	&PID_Calc,\
	&PID_Reset,\
}

typedef enum
{
	START_STATE,
	PREPARE_STATE,     	
	NORMAL_STATE,		  
  DEFEND_STATE,
  ATTACK_STATE,  
	STOP_STATE        
}WorkState_e;

extern WorkState_e WorkState;
extern uint8_t find_enemy;
extern uint16_t enemy_pitch;
extern uint16_t enemy_yaw;
extern uint16_t manifold_fine_cnt;
extern uint8_t target_hero;

extern int16_t yawIntensity ;

extern uint16_t maincnt;

extern fw_PID_Regulator_t pitchPositionPID;
extern fw_PID_Regulator_t yawPositionPID;
extern fw_PID_Regulator_t pitchSpeedPID;
extern fw_PID_Regulator_t yawSpeedPID;

extern double bullet_angle_target;
extern double bullet2_angle_target;
extern float yawRealAngle;
extern float pitchRealAngle;
extern float auto_attack_yaw_kp;
extern float auto_attack_pitch_kp;
extern float auto_attack_yaw_kd;
extern float auto_attack_pitch_kd;

extern unsigned int enemy_lost;

extern float friclSpeedTarget;
extern float fricrSpeedTarget;

void CMControlInit(void);

#endif /*__ CONTROLTASK_H */
