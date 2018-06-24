/**
  ******************************************************************************
  * File Name          : RemoteTask.c
  * Description        : 遥控器处理任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"
uint8_t rc_data[18];
RC_Ctl_t RC_CtrlData;
InputMode_e inputmode = REMOTE_INPUT; 
ChassisSpeed_Ref_t ChassisSpeedRef; 
float bullet_ref = 0;
float bullet2_ref = 0;
extern FrictionWheelState_e FrictionWheelState;
RemoteSwitch_t g_switch1;
extern RampGen_t frictionRamp ;  //摩擦轮斜坡
extern RampGen_t LRSpeedRamp ;   //键盘速度斜坡
extern RampGen_t FBSpeedRamp  ;   

float yawSpeedTarget = 0.0;
float yawAngleTarget = 0.0;
float pitchAngleTarget = 0.0;

//遥控器控制量初始化
void RemoteTaskInit()
{
	frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
	frictionRamp.ResetCounter(&frictionRamp);
	
	yawSpeedTarget = 0.0;
	pitchAngleTarget = 0.0;
	/*底盘速度初始化*/
	ChassisSpeedRef.forward_back_ref = 0.0f;
	/*摩擦轮*/
	FrictionWheelState = FRICTION_WHEEL_OFF;
}

//摇杆控制量解算
void RemoteControlProcess(Remote *rc)
{
	if(WorkState == NORMAL_STATE)
	{
		ChassisSpeedRef.forward_back_ref = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
		//ChassisSpeedRef.left_right_ref   = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT; 
		bullet_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 1.5;
		//bullet2_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 1.5;
		//bullet_angle_target += (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
		//bullet2_angle_target += (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
		
		pitchAngleTarget += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
		yawSpeedTarget = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_SPEED_INC_FACT;
	}
	RemoteShootControl(&g_switch1, rc->s1);
}

//摇杆控制量解算
void RemoteControlProcessAuto(Remote *rc)
{
	if(WorkState == DEFEND_STATE || WorkState == ATTACK_STATE)
	{
		bullet_ref = (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 0.2;
		bullet2_ref = (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT * 1.5;
		//bullet_angle_target += (rc->ch1 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
		//bullet2_angle_target += (rc->ch0 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_BULLET_POSITION_REF_FACT;
	}
	//RemoteShootControl(&g_switch1, rc->s1);
}

/*拨杆数据处理*/   
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
	static uint32_t switch_cnt = 0;

	sw->switch_value_raw = val;
	sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;

	//value1 value2的值其实是一样的
	//value1高4位始终为0
	sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
	(sw->switch_value_buf[sw->buf_index]);

	sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

	sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;	

	//如果两次数据一样，即没有更新数据，拨杆不动
	if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
	{
		switch_cnt++;	
	}
	else
	{
		switch_cnt = 0;
	}
	//如果拨杆维持了一定时间，即连续来了40帧一样的数据，则把拨杆数据写入switch_long_value
	if(switch_cnt >= 40)
	{
		sw->switch_long_value = sw->switch_value_buf[sw->buf_index]; 	
	}
	//指向下一个缓冲区
	sw->buf_last_index = sw->buf_index;
	sw->buf_index++;		
	if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
	{
		sw->buf_index = 0;	
	}			
}


//遥控器数据解算
void RemoteDataProcess(uint8_t *pData)
{
	if(pData == NULL)
	{
			return;
	}
	//遥控器 11*4 + 2*2 = 48，需要 6 Bytes
	//16位，只看低11位
	RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
											 ((int16_t)pData[4] << 10)) & 0x07FF;
	RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
	
	//16位，只看最低两位
	RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);

	//鼠标需要 8 Bytes
	RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);    

	RC_CtrlData.mouse.press_l = pData[12];
	RC_CtrlData.mouse.press_r = pData[13];
	
	//键盘需要 2 Bytes = 16 bits ，每一位对应一个键
	RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);

	//输入状态设置
	if(RC_CtrlData.rc.s2 == 1) inputmode = REMOTE_INPUT; 
	else if(RC_CtrlData.rc.s2 == 3) inputmode = AUTO; 
	else inputmode = STOP; 
	
	/*左上角拨杆状态（RC_CtrlData.rc.s1）获取*/	//用于遥控器发射控制
	GetRemoteSwitchAction(&g_switch1, RC_CtrlData.rc.s1);
	
	switch(inputmode)
	{
		case REMOTE_INPUT:               
		{
			if(WorkState == NORMAL_STATE)
			{ 
				RemoteControlProcess(&(RC_CtrlData.rc));
			}
		}break;
		case AUTO:              
		{
			RemoteControlProcessAuto(&(RC_CtrlData.rc));
		}break;
		case STOP:               
		{
			 
		}break;
	}
}

//初始化遥控器串口DMA接收
void InitRemoteControl(){
	if(HAL_UART_Receive_DMA(&RC_UART, rc_data, 18) != HAL_OK){
			Error_Handler();
	} 
	RemoteTaskInit();
}

//遥控器串口中断入口函数，从此处开始执行
uint8_t rc_first_frame = 0;
uint8_t rc_update = 0;
uint8_t rc_cnt = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	if(UartHandle == &RC_UART){
		rc_update = 1;
	}
	else if(UartHandle == &MANIFOLD_UART)
	{
		manifoldUartRxCpltCallback();  //妙算信号数据解算
		//#ifdef DEBUG_MODE
		//ctrlUartRxCpltCallback();
		//#endif
	}
	else if(UartHandle == &JUDGE_UART)
	{
		//manifoldUartRxCpltCallback();
		//judgeUartRxCpltCallback();  //裁判系统数据解算
		//#ifdef DEBUG_MODE
		//ctrlUartRxCpltCallback();
		//#endif
	}
}   
