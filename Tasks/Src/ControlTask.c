/**
  ******************************************************************************
  * File Name          : ControlTask.c
  * Description        : 主控制任务
  ******************************************************************************
  *
  * Copyright (c) 2018 Team TPP-Shanghai Jiao Tong University
  * All rights reserved.
  *
  ******************************************************************************
  */
#include "includes.h"

#define LED_GREEN_TOGGLE() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define LED_RED_TOGGLE()   HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define LED_GREEN_OFF()     HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_SET)
#define LED_RED_OFF()       HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_SET)
#define LED_GREEN_ON()    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,GPIO_PIN_RESET)
#define LED_RED_ON()      HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,GPIO_PIN_RESET)

uint8_t find_enemy = 0;
uint8_t on_enemy = 0;
uint8_t target_hero = 0;
uint16_t yaw_offset = 320;
uint16_t pitch_offset = 210;
uint16_t enemy_yaw = 320;
uint16_t enemy_pitch = 210;
uint16_t manifold_fine_cnt = 0;
WorkState_e WorkState = START_STATE;
uint16_t prepare_time = 0;

uint8_t bulletshooted = 0;
uint16_t bulletshootedcnt = 0;
uint8_t nobullet = 0;
uint8_t bulletSpeedBuf_last[4] = {0};

PID_Regulator_t CMRotatePID = CHASSIS_MOTOR_ROTATE_PID_DEFAULT; 
PID_Regulator_t CM1SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t CM2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t BulletSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t Bullet2SpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

float friclSpeedTarget = 0;
float fricrSpeedTarget = 0;
PID_Regulator_t FRICLSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;
PID_Regulator_t FRICRSpeedPID = CHASSIS_MOTOR_SPEED_PID_DEFAULT;

//PID_Regulator_t BulletPositionPID = BULLET_POSITION_PID_DEFAULT;
//PID_Regulator_t Bullet2PositionPID = BULLET_POSITION_PID_DEFAULT;
//PID_Regulator_t BulletSpeedPID = BULLET_SPEED_PID_DEFAULT;//0.0, 0.00003
//PID_Regulator_t Bullet2SpeedPID = BULLET_SPEED_PID_DEFAULT;//0.0, 0.00003

int16_t CMFLIntensity = 0, CMFRIntensity = 0, BulletIntensity = 0,Bullet2Intensity = 0, FRICLIntensity = 0, FRICRIntensity = 0;
int16_t yawIntensity = 0;		
int16_t pitchIntensity = 0;

//底盘PID初始化
void CMControlInit(void)
{
	CMRotatePID.Reset(&CMRotatePID);
	CM1SpeedPID.Reset(&CM1SpeedPID);
	CM2SpeedPID.Reset(&CM2SpeedPID);
	FRICLSpeedPID.Reset(&FRICLSpeedPID);
	FRICRSpeedPID.Reset(&FRICRSpeedPID);
	BulletSpeedPID.Reset(&BulletSpeedPID);
	Bullet2SpeedPID.Reset(&Bullet2SpeedPID);
	//BulletPositionPID.Reset(&BulletPositionPID);
	//Bullet2PositionPID.Reset(&Bullet2PositionPID);
}

//单个底盘电机的控制，下同
void ControlCMFL(void)
{		
	CM1SpeedPID.ref =  ChassisSpeedRef.forward_back_ref*0.075+ ChassisSpeedRef.left_right_ref*0.075;
	CM1SpeedPID.ref = 160 * CM1SpeedPID.ref;	
			
	CM1SpeedPID.fdb = CMFLRx.RotateSpeed;

	CM1SpeedPID.Calc(&CM1SpeedPID);
	CMFLIntensity = CHASSIS_SPEED_ATTENUATION * CM1SpeedPID.output;
}

void ControlCMFR(void)
{		
	CM2SpeedPID.ref = - ChassisSpeedRef.forward_back_ref*0.075 + ChassisSpeedRef.left_right_ref*0.075;
	CM2SpeedPID.ref = 160 * CM2SpeedPID.ref;	
			
	CM2SpeedPID.fdb = CMFRRx.RotateSpeed;

	CM2SpeedPID.Calc(&CM2SpeedPID);
	CMFRIntensity = CHASSIS_SPEED_ATTENUATION * CM2SpeedPID.output;
}

void ControlFRICL(void)
{		
	FRICLSpeedPID.ref =  friclSpeedTarget*0.075;
	FRICLSpeedPID.ref = 160 * FRICLSpeedPID.ref;	
			
	FRICLSpeedPID.fdb = FRICLRx.RotateSpeed;

	FRICLSpeedPID.Calc(&FRICLSpeedPID);
	FRICLIntensity = CHASSIS_SPEED_ATTENUATION * FRICLSpeedPID.output;
}

void ControlFRICR(void)
{		
	FRICRSpeedPID.ref =  fricrSpeedTarget*0.075;
	FRICRSpeedPID.ref = 160 * FRICRSpeedPID.ref;	
			
	FRICRSpeedPID.fdb = FRICRRx.RotateSpeed;

	FRICRSpeedPID.Calc(&FRICRSpeedPID);
	FRICRIntensity = CHASSIS_SPEED_ATTENUATION * FRICRSpeedPID.output;
}

void ControlBullet(void)
{		
	BulletSpeedPID.kp = 3.0;
	BulletSpeedPID.kd = 0.0;
	BulletSpeedPID.ref = bullet_ref*0.075;
	BulletSpeedPID.ref = 160 * BulletSpeedPID.ref;	
			
	BulletSpeedPID.fdb = BulletRx.RotateSpeed;

	BulletSpeedPID.Calc(&BulletSpeedPID);
	BulletIntensity = CHASSIS_SPEED_ATTENUATION * BulletSpeedPID.output;
}

void ControlBullet2(void)
{		
	Bullet2SpeedPID.kp = 3.0;
	Bullet2SpeedPID.kd = 0.0;
	if(bullet2_ref<300 && bullet2_ref>-300) bullet2_ref = 0;
	Bullet2SpeedPID.ref = bullet2_ref*0.075;
	Bullet2SpeedPID.ref = 160 * Bullet2SpeedPID.ref;	
			
	Bullet2SpeedPID.fdb = Bullet2Rx.RotateSpeed;

	Bullet2SpeedPID.Calc(&Bullet2SpeedPID);
	Bullet2Intensity = CHASSIS_SPEED_ATTENUATION * Bullet2SpeedPID.output;
}

double bullet_angle_target=0;
double bulletRealAngle=0;
double bullet_zero_angle=0;
//void setBulletWithAngle(double targetAngle){//360.0 * 12 * 2
//		static double AngleLast = 180.0;
//		double AngleCurr = BulletRx.angle * 360 / 8192.0;
//		static uint8_t isInitiated=0;
//		if(isInitiated==0)
//		{
//			bulletRealAngle=AngleCurr;
//			bullet_zero_angle=AngleCurr;
//			AngleLast=AngleCurr;
//			isInitiated=1;
//			return;
//		}
//		if(AngleCurr - AngleLast > 180){
//			bulletRealAngle += AngleCurr - 360 - AngleLast;
//		}else if(AngleCurr - AngleLast < -180){
//			bulletRealAngle += AngleCurr + 360 - AngleLast;
//		}else{
//			bulletRealAngle += AngleCurr - AngleLast;
//		}
//		AngleLast=AngleCurr;
//		//RealSpeed
//		double realSpeed = BulletRx.RotateSpeed * 6;//度每秒(* 360 / 60.0)

//		BulletPositionPID.ref = targetAngle;
//		BulletPositionPID.fdb = bulletRealAngle;
//		BulletPositionPID.Calc(&BulletPositionPID);
//		
//		BulletSpeedPID.ref = BulletPositionPID.output;
//		BulletSpeedPID.fdb = realSpeed;
//		BulletSpeedPID.Calc(&BulletSpeedPID);
//		BulletIntensity = BulletSpeedPID.output;
//}

double bullet2_angle_target=0;
double bullet2RealAngle=0;
double bullet2_zero_angle=0;
//void setBullet2WithAngle(double targetAngle){//360.0 * 12 * 2
//		static double AngleLast = 180.0;
//		double AngleCurr = Bullet2Rx.angle * 360 / 8192.0;
//		static uint8_t isInitiated=0;
//		if(isInitiated==0)
//		{
//			bullet2RealAngle=AngleCurr;
//			bullet2_zero_angle=AngleCurr;
//			AngleLast=AngleCurr;
//			isInitiated=1;
//			return;
//		}
//		if(AngleCurr - AngleLast > 180){
//			bullet2RealAngle += AngleCurr - 360 - AngleLast;
//		}else if(AngleCurr - AngleLast < -180){
//			bullet2RealAngle += AngleCurr + 360 - AngleLast;
//		}else{
//			bullet2RealAngle += AngleCurr - AngleLast;
//		}
//		AngleLast=AngleCurr;
//		//RealSpeed
//		double realSpeed = Bullet2Rx.RotateSpeed * 6;//度每秒(* 360 / 60.0)

//		Bullet2PositionPID.ref = targetAngle;
//		Bullet2PositionPID.fdb = bullet2RealAngle;
//		Bullet2PositionPID.Calc(&Bullet2PositionPID);
//		
//		Bullet2SpeedPID.ref = Bullet2PositionPID.output;
//		Bullet2SpeedPID.fdb = realSpeed;
//		Bullet2SpeedPID.Calc(&Bullet2SpeedPID);
//		Bullet2Intensity = Bullet2SpeedPID.output;
//}
unsigned char testred1 = 0;
unsigned char testred2 = 0;
unsigned char testred3 = 0;
unsigned char testred4 = 0;
unsigned char calicnt = 0;
unsigned int downcnt = 0;
unsigned int upcnt = 0;

float odometry = 0.0;
float odometryatt = 0.0;
void odometryLoop()
{
	testred1 = redBuf & 0x01;
	testred2 = (redBuf & 0x02)>>1;
	testred3 = (redBuf & 0x04)>>2;
	testred4 = (redBuf & 0x08)>>3;
	
	if(testred1 == 0)
	{
		if(calicnt>3) odometry = 0.0;
		else calicnt++;
	}
	else calicnt = 0;
	
	if(testred2 == 0)
	{
		if(downcnt>3) odometry = -500000.0;
		else downcnt++;
	}
	else downcnt = 0;
	
	if(testred4 == 0)
	{
		if(upcnt>3) odometry = 500000.0;
		else upcnt++;
	}
	else upcnt = 0;
	
	if((CMFLRx.RotateSpeed > 50 || CMFLRx.RotateSpeed < -50) && (CMFRRx.RotateSpeed > 50 || CMFRRx.RotateSpeed < -50))
	{
		odometry += (CMFLRx.RotateSpeed - CMFRRx.RotateSpeed) * ODOMETRY_FACT;
	}
}

extern FrictionWheelState_e FrictionWheelState;
extern Shoot_State_e ShootState;
unsigned int enemy_lost = 0;
uint8_t enemy_far = 0;

uint8_t pitch_dir = 0;
uint8_t up_dir = 0;
//状态机切换
void WorkStateFSM(void)
{
	static int blink_cnt = 0;
	blink_cnt++;
	switch (WorkState)
	{
		case START_STATE:
		{
			if(prepare_time<2000) prepare_time++;
			if(prepare_time == 2000)
			{
				WorkState = PREPARE_STATE;
			}
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case PREPARE_STATE:
		{
			if(prepare_time<4000) prepare_time++;
			if(prepare_time == 4000)
			{
				WorkState = NORMAL_STATE;
				prepare_time = 0;
				LED_GREEN_OFF();
				LED_RED_OFF();
				blink_cnt = 0;
			}
			
			if (inputmode == STOP) WorkState = STOP_STATE;
		}break;
		case NORMAL_STATE:
		{
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(0); 
				FrictionWheelState = FRICTION_WHEEL_OFF;
			}
			else if (inputmode == AUTO)
			{
				WorkState = DEFEND_STATE;
			}
			
			if(gameProgress == 4)
			{
				SetFrictionWheelSpeed(400); 
				WorkState = DEFEND_STATE;
				FrictionWheelState = FRICTION_WHEEL_ON;
			}
			
			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				LED_GREEN_TOGGLE();
			}
		}break;
		case DEFEND_STATE:  
		{
			if (find_enemy == 1 && nobullet == 0 && enemy_far == 0) 
			{
				WorkState = ATTACK_STATE;
				odometryatt = odometry;
			}
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(0); 
				FrictionWheelState = FRICTION_WHEEL_OFF;
				frictionRamp.ResetCounter(&frictionRamp);
				bulletshootedcnt = 0;
				nobullet = 0;
				find_enemy = 0;
				enemy_yaw = 320;
				enemy_pitch = 210;
				bullet_ref = 0;
			}

			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
				LED_RED_TOGGLE();
			}
		}break;
		case ATTACK_STATE: 
		{
			if (nobullet == 1)  WorkState = DEFEND_STATE;
			
			if(manifold_fine_cnt>500)   
			{
				WorkState = DEFEND_STATE;
			}
			else
			{
				manifold_fine_cnt++;
			}
			
			if (find_enemy == 0 || enemy_far == 1) 
			{
				enemy_lost++;
				if (enemy_lost > 2000) 
				{
					WorkState = DEFEND_STATE;
					enemy_lost = 0;
					pitch_dir= 0;
				}
			}
			else enemy_lost = 0;
			
			if (inputmode == STOP) 
			{
				WorkState = STOP_STATE;
				SetFrictionWheelSpeed(0); 
				frictionRamp.ResetCounter(&frictionRamp);
				FrictionWheelState = FRICTION_WHEEL_OFF;
				bulletshootedcnt = 0;
				nobullet = 0;
				find_enemy = 0;
				enemy_yaw = 320;
				enemy_pitch = 210;
				bullet_ref = 0;
			}

			if (blink_cnt == 1000) 
			{
				blink_cnt = 0;
			}
		}break;
		case STOP_STATE:
		{
			if (inputmode == REMOTE_INPUT)
			{
				WorkState = PREPARE_STATE;
				RemoteTaskInit();
			}
		}break;
	}
}

//底盘电机CAN信号控制
void setCMMotor()
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = CM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	CMGMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(CMFLIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)CMFLIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(CMFRIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)CMFRIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = (uint8_t)(BulletIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[5] = (uint8_t)BulletIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = (uint8_t)(Bullet2Intensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[7] = (uint8_t)Bullet2Intensity;
	
	if(can1_update == 1 && can1_type == 0)
	{
		//CAN通信前关中断
		HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	  HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_DisableIRQ(USART3_IRQn);
		HAL_NVIC_DisableIRQ(USART6_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		//CAN通信后开中断，防止中断影响CAN信号发送
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
		HAL_NVIC_EnableIRQ(USART6_IRQn);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}

//云台电机CAN信号控制
void setGMMotor()
{
	CanTxMsgTypeDef pData;
	CMGMMOTOR_CAN.pTxMsg = &pData;
	
	CMGMMOTOR_CAN.pTxMsg->StdId = GM_TXID;
	CMGMMOTOR_CAN.pTxMsg->ExtId = 0;
	CMGMMOTOR_CAN.pTxMsg->IDE = CAN_ID_STD;
	CMGMMOTOR_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	CMGMMOTOR_CAN.pTxMsg->DLC = 0x08;
	
	CMGMMOTOR_CAN.pTxMsg->Data[0] = (uint8_t)(pitchIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[1] = (uint8_t)pitchIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[2] = (uint8_t)(yawIntensity >> 8);
	CMGMMOTOR_CAN.pTxMsg->Data[3] = (uint8_t)yawIntensity;
	CMGMMOTOR_CAN.pTxMsg->Data[4] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[5] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[6] = 0;
	CMGMMOTOR_CAN.pTxMsg->Data[7] = 0;

	if(can1_update == 1 && can1_type == 1)
	{
		HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	  HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_DisableIRQ(USART3_IRQn);
		HAL_NVIC_DisableIRQ(USART6_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&CMGMMOTOR_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can1_update = 0;
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
		HAL_NVIC_EnableIRQ(USART6_IRQn);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}

//摩擦轮电机CAN信号控制
void setFRICMotor()
{
	CanTxMsgTypeDef pData;
	FRIC_CAN.pTxMsg = &pData;
	
	FRIC_CAN.pTxMsg->StdId = FR_TXID;
	FRIC_CAN.pTxMsg->ExtId = 0;
	FRIC_CAN.pTxMsg->IDE = CAN_ID_STD;
	FRIC_CAN.pTxMsg->RTR = CAN_RTR_DATA;
	FRIC_CAN.pTxMsg->DLC = 0x08;
	
	FRIC_CAN.pTxMsg->Data[0] = (uint8_t)(FRICLIntensity >> 8);
	FRIC_CAN.pTxMsg->Data[1] = (uint8_t)FRICLIntensity;
	FRIC_CAN.pTxMsg->Data[2] = (uint8_t)(FRICRIntensity >> 8);
	FRIC_CAN.pTxMsg->Data[3] = (uint8_t)FRICRIntensity;
	FRIC_CAN.pTxMsg->Data[4] = 0;
	FRIC_CAN.pTxMsg->Data[5] = 0;
	FRIC_CAN.pTxMsg->Data[6] = 0;
	FRIC_CAN.pTxMsg->Data[7] = 0;
	
	if(can2_update == 1)
	{
		//CAN通信前关中断
		HAL_NVIC_DisableIRQ(DMA1_Stream1_IRQn);
	  HAL_NVIC_DisableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_DisableIRQ(USART3_IRQn);
		HAL_NVIC_DisableIRQ(USART6_IRQn);
		HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
		if(HAL_CAN_Transmit_IT(&FRIC_CAN) != HAL_OK)
		{
			Error_Handler();
		}
		can2_update = 0;
		//CAN通信后开中断，防止中断影响CAN信号发送
		HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
		HAL_NVIC_EnableIRQ(USART3_IRQn);
		HAL_NVIC_EnableIRQ(USART6_IRQn);
		HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_EnableIRQ(TIM7_IRQn);
		#ifdef DEBUG_MODE
			HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
		#endif
	}
}

#define NORMALIZE_ANGLE180(angle) angle = ((angle) > 180) ? ((angle) - 360) : (((angle) < -180) ? (angle) + 360 : angle)
fw_PID_Regulator_t pitchPositionPID = fw_PID_INIT(7.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t yawPositionPID = fw_PID_INIT(5.0, 0.0, 0.5, 10000.0, 10000.0, 10000.0, 10000.0);
fw_PID_Regulator_t pitchSpeedPID = fw_PID_INIT(20.0, 0.0, 0.0, 10000.0, 10000.0, 10000.0, 3500.0); //KP²»ÄÜ³¬¹ý30
fw_PID_Regulator_t yawSpeedPID = fw_PID_INIT(10.0, 0.0, 0, 10000.0, 10000.0, 10000.0, 2000.0);
#define yaw_zero 7200  
#define pitch_zero 5796
float yawRealAngle = 0.0;
float pitchRealAngle = 0.0;
float gap_angle = 0.0;

//控制云台YAW轴
void ControlYawSpeed(void)
{	
	yawIntensity = ProcessYawPID(yawSpeedTarget,-gYroZs);
}

//控制云台pitch轴
void ControlPitch(void)
{
	uint16_t pitchZeroAngle = pitch_zero;
				
	pitchRealAngle = -(GMPITCHRx.angle - pitchZeroAngle) * 360 / 8192.0;
	NORMALIZE_ANGLE180(pitchRealAngle);

	MINMAX(pitchAngleTarget, PITCHANGLETARGETMIN1, PITCHANGLETARGETMAX1);
				
	pitchIntensity = -ProcessPitchPID(pitchAngleTarget,pitchRealAngle,gYroYs);
}

float enemy_yaw_err = 0;
float enemy_yaw_out = 0;
float enemy_pitch_err = 0;
float enemy_pitch_out = 0;

float pitchAngleTargetMin = 0;
float yawSpeed = 200;
void Defend_Action()
{
	if(odometry > -170000 && odometry < -140000)
	{
		pitchAngleTargetMin = PITCHANGLETARGETMIN2;
	}
	else 
	{
		pitchAngleTargetMin = PITCHANGLETARGETMIN1;
	}
	
	if(pitchAngleTarget > (PITCHANGLETARGETMAX1 - 0.5))
	{
		pitch_dir = 0;
	}
	else if(pitchAngleTarget < (pitchAngleTargetMin + 0.5))
	{
		pitch_dir = 1;
	}
	
	if (pitch_dir == 0)
	{
		pitchAngleTarget -= PITCH_DEFEND_SPEED;
	}
	else if (pitch_dir == 1)
	{
		pitchAngleTarget += PITCH_DEFEND_SPEED;
	}
	
	yawSpeedTarget = YAW_DEFEND_SPEED;
	
	if(odometry < ODOMETRY_DOWNMAX1) up_dir = 0;
	if(odometry > ODOMETRY_UPMAX1) up_dir = 1;
	
	if(odometry > -170000 && odometry < -140000 && nobullet == 0)
	{
		if(up_dir == 0) 
		{
			if (ChassisSpeedRef.forward_back_ref > ODOMETRY_SPEED3) 
			{
				ChassisSpeedRef.forward_back_ref -= 4;
			}
			else ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED3;
		}
		else if(up_dir == 1) 
		{
			if (ChassisSpeedRef.forward_back_ref < -ODOMETRY_SPEED3)
			{
				ChassisSpeedRef.forward_back_ref += 4;
			}
			else ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED3;
		}
	}
	else 
	{
		if(up_dir == 0) 
		{
			if (ChassisSpeedRef.forward_back_ref < ODOMETRY_SPEED1) 
			{
				if(odometry < -400000) ChassisSpeedRef.forward_back_ref += 10;
				else ChassisSpeedRef.forward_back_ref += 0.4;
			}
			else ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED1;
		}
		else if(up_dir == 1) 
		{
			if (ChassisSpeedRef.forward_back_ref > -ODOMETRY_SPEED1)
			{
				if(odometry > 400000) ChassisSpeedRef.forward_back_ref -= 10;
				else ChassisSpeedRef.forward_back_ref -= 0.4;
			}
			else ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED1;
		}
	}
	
	bullet_ref = 0;
}

uint8_t catchedcnt =  0 ;
uint8_t up_diratt = 0;
float pitchAngleTargetMax = PITCHANGLETARGETMAX1;
void Attack_Action()
{
	if(odometry < (odometryatt + ODOMETRY_DOWNMAX2) || odometry < ODOMETRY_DOWNMAX2) ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED2;
	if(odometry > (odometryatt + ODOMETRY_UPMAX2) || odometry > ODOMETRY_UPMAX2) ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED2;
	
//	if(shooterHeat0 > STOPHEAT) 
//	{
//		if(odometry < (odometryatt + ODOMETRY_DOWNMAX2) || odometry < ODOMETRY_DOWNMAX1) up_diratt = 0;
//		if(odometry > (odometryatt + ODOMETRY_UPMAX2) || odometry > ODOMETRY_UPMAX1) up_diratt = 1;
//		
//		if(up_diratt == 0) 
//		{
//			ChassisSpeedRef.forward_back_ref = ODOMETRY_SPEED2;
//		}
//		else if(up_diratt == 1) 
//		{
//			ChassisSpeedRef.forward_back_ref = -ODOMETRY_SPEED2;
//		}
//	}
//	else if (shooterHeat0 < 150) 
	ChassisSpeedRef.forward_back_ref = 0.0;
		
	static float enemy_yaw_err_last = 0;
	enemy_yaw_err = (float)(yaw_offset - enemy_yaw);
	enemy_yaw_out = enemy_yaw_err *  AUTO_ATTACK_YAW_KP + (enemy_yaw_err - enemy_yaw_err_last)*AUTO_ATTACK_YAW_KD;
	if (enemy_yaw_out>100) enemy_yaw_out = 100;
	else if (enemy_yaw_out<-100) enemy_yaw_out = -100;
	yawSpeedTarget = enemy_yaw_out;
	enemy_yaw_err_last = enemy_yaw_err;
		
	static float enemy_pitch_err_last = 0;
	enemy_pitch_err = (float)(pitch_offset - enemy_pitch);
	enemy_pitch_out = enemy_pitch_err * AUTO_ATTACK_PITCH_KP + (enemy_pitch_err - enemy_pitch_err_last)*AUTO_ATTACK_PITCH_KD;
	if (enemy_pitch_out>1) enemy_pitch_out = 1;
	else if (enemy_pitch_out<-1) enemy_pitch_out = -1;
	pitchAngleTarget -= enemy_pitch_out;
	enemy_pitch_err_last = enemy_pitch_err;
	if(target_hero == 0) 
	{
		if (pitchAngleTarget > PITCHANGLETARGETMAX2) pitchAngleTarget = PITCHANGLETARGETMAX2;
	}
		
	if(enemy_yaw_err<50 && enemy_yaw_err>-50 && enemy_pitch_err<60 && enemy_pitch_err>-60) 
	{
		if (catchedcnt > 10)
		{
			if(shooterHeat0 < 100) bullet_ref = BULLET_SPEED; 
		}
		else catchedcnt++;
	}
	else 
	{
		catchedcnt = 0;
		bullet_ref = 0;
	}
}
//主控制循环
void controlLoop()
{
	if(target_hero == 0) pitchAngleTargetMax = PITCHANGLETARGETMAX2;
	else pitchAngleTargetMax = PITCHANGLETARGETMAX1;
	
	if((enemy_pitch > ((pitchAngleTargetMax-pitchAngleTarget) * 3 + 310)) && pitchAngleTarget >20) enemy_far = 1;
	else enemy_far = 0;
	
	if(target_hero == 0) 
	{
		if(pitchAngleTarget < 30) pitch_offset = 210;
		else if(pitchAngleTarget < 40) pitch_offset = (unsigned int)(210 - (pitchAngleTarget - 30));
		else if(pitchAngleTarget < 45) pitch_offset = (unsigned int)(200 - (pitchAngleTarget - 40)*2);
		else if(pitchAngleTarget < 50) pitch_offset = (unsigned int)(188 - (pitchAngleTarget - 45));
		else if(pitchAngleTarget < 55) pitch_offset = (unsigned int)(183 + (pitchAngleTarget - 50));
		else if(pitchAngleTarget < (PITCHANGLETARGETMAX1 + 0.1)) pitch_offset = 188;
	}
	else pitch_offset = 210;
		
	WorkStateFSM();
	
	if(WorkState != START_STATE) 
	{
		odometryLoop();
		
		if(bulletSpeedBuf[0] != bulletSpeedBuf_last[0] || bulletSpeedBuf[1] != bulletSpeedBuf_last[1] || bulletSpeedBuf[2] != bulletSpeedBuf_last[2] || bulletSpeedBuf[3] != bulletSpeedBuf_last[3])
		{
			bulletshooted = 1;
			bulletshootedcnt = 0;
			nobullet = 0;
		}
		else bulletshooted = 0;
		bulletSpeedBuf_last[0] = bulletSpeedBuf[0];
		bulletSpeedBuf_last[1] = bulletSpeedBuf[1];
		bulletSpeedBuf_last[2] = bulletSpeedBuf[2];
		bulletSpeedBuf_last[3] = bulletSpeedBuf[3];
		
		if(WorkState == DEFEND_STATE) Defend_Action();
		else if(WorkState == ATTACK_STATE) Attack_Action();
		
		ControlYawSpeed();
		ControlPitch();
		
		if(WorkState == STOP_STATE)
		{
			yawIntensity = 0;
			pitchIntensity = 0;
		}
		setGMMotor();
		
		ControlCMFL();
		ControlCMFR();
		
		if(shooterHeat0 > STOPHEAT) bullet_ref = 0;
		ControlBullet();
		
		if (bullet_ref > 100) 
		{
			if(bulletshootedcnt < 20000) bulletshootedcnt++;
			else nobullet = 1;
		}
		
		if(WorkState == STOP_STATE)
		{
			CMFLIntensity = 0;
			CMFRIntensity = 0;
			BulletIntensity = 0;
			Bullet2Intensity = 0;
		}
		setCMMotor();
		
		ControlFRICL();
		ControlFRICR();
		if(WorkState == STOP_STATE)
		{
			FRICLIntensity = 0;
			FRICRIntensity = 0;
		}
		setFRICMotor();
	}
}

//时间中断入口函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == htim6.Instance)
	{
		//主循环在时间中断中启动
		controlLoop();
	}
	else if (htim->Instance == htim7.Instance)
	{
		rc_cnt++;
		if (rc_update)
		{
			if( (rc_cnt <= 17) && (rc_first_frame == 1))
			{
				RemoteDataProcess(rc_data);				//遥控器数据解算
		    HAL_UART_AbortReceive(&RC_UART);
		    HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
			}
			else
			{
				if(rc_first_frame) WorkState = PREPARE_STATE;
				HAL_UART_AbortReceive(&RC_UART);
		    HAL_UART_Receive_DMA(&RC_UART, rc_data, 18);
				rc_cnt = 0;
				rc_first_frame = 1;
			}
			rc_update = 0;
		}
	}
	#ifdef DEBUG_MODE
	else if (htim->Instance == htim10.Instance)  //10ms，处理上位机数据，优先级不高
	{
		zykProcessData();
	}
	#endif
}