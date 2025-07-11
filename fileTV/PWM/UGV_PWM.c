#include "stm32f4xx.h"
#include "UGV_ENC.h"
#include "UGV_PWM.h"


/**
 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   PWM Pin define 
		*/
		#define		PWM_CLK_Cmd		  								 		RCC_AHB1PeriphClockCmd
		#define		PWM_CLK															RCC_AHB1Periph_GPIOE
		
		#define 	PWM_CH1_PORT 					 						 	GPIOE
		#define		PWM_CH1_PIN													GPIO_Pin_9
		#define 	PWM_CH1_SOURCE									 		GPIO_PinSource9
		
		#define 	PWM_CH1N_PORT 					 						GPIOE
		#define		PWM_CH1N_PIN												GPIO_Pin_8
		#define 	PWM_CH1N_SOURCE									 		GPIO_PinSource8

    #define 	PWM_CH2_PORT 					 						 	GPIOE
		#define		PWM_CH2_PIN													GPIO_Pin_11
		#define 	PWM_CH2_SOURCE									 		GPIO_PinSource11
		
		#define 	PWM_CH2N_PORT 					 						GPIOE
		#define		PWM_CH2N_PIN												GPIO_Pin_10
		#define 	PWM_CH2N_SOURCE									 		GPIO_PinSource10
		
		#define		PWM_TIM_AF													GPIO_AF_TIM1
		#define		PWM_TIM															TIM1
		#define 	PWM_TIM_CLK													RCC_APB2Periph_TIM1
		#define		PWM_TIM_CLK_Cmd											RCC_APB2PeriphClockCmd
		#define		PWM_TIM_Prescaler										0
		#define		PWM_TIM_Period											8399
		#define		PWM_TIM_ClockDivision								0
		#define		PWM_TIM_RepetitionCounter						0
		#define		PWM_TIM_Deadtime										100
/**
 * @}
 */

float vel_left,vel_right,filtered_speed_left,filtered_speed_right;
extern float Kp1,Ki1,Kd1,Kp2,Ki2,Kd2;
MOTOR Motor_1;
MOTOR Motor_2;
CAR Car;

void Motor_Config(void)
{
	GPIO_InitTypeDef 					GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef 				TIM_OCInitStructure;
	TIM_BDTRInitTypeDef 			TIM_BDTRInitStructure;

	/* Enable GPIO clock */
	PWM_CLK_Cmd(PWM_CLK, ENABLE);
	/* TIM1 clock enable */
  PWM_TIM_CLK_Cmd(PWM_TIM_CLK, ENABLE);
	
	/* GPIOE Configuration: Channel 2 as alternate function push-pull */ 
  GPIO_InitStructure.GPIO_Pin = PWM_CH1N_PIN | PWM_CH1_PIN | PWM_CH2N_PIN | PWM_CH2_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	
  GPIO_Init(GPIOE, &GPIO_InitStructure); 
	
  GPIO_PinAFConfig(PWM_CH1_PORT , PWM_CH1_SOURCE , PWM_TIM_AF);
	GPIO_PinAFConfig(PWM_CH1N_PORT, PWM_CH1N_SOURCE, PWM_TIM_AF);
  GPIO_PinAFConfig(PWM_CH2_PORT , PWM_CH2_SOURCE , PWM_TIM_AF);
	GPIO_PinAFConfig(PWM_CH2N_PORT, PWM_CH2N_SOURCE, PWM_TIM_AF);
	
	/* Time Base configuration */
  TIM_TimeBaseStructure.TIM_Prescaler = PWM_TIM_Prescaler;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseStructure.TIM_Period = PWM_TIM_Period;
  TIM_TimeBaseStructure.TIM_ClockDivision = PWM_TIM_ClockDivision;
  TIM_TimeBaseStructure.TIM_RepetitionCounter = PWM_TIM_RepetitionCounter;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
	
	/* Channel 1 and 2 Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0; /* TIM1->CCR1;*/
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);

  TIM_OCInitStructure.TIM_Pulse = 0; /* TIM1->CCR2;*/
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	
	  /* Automatic Output enable, Break, dead time and lock configuration*/
  TIM_BDTRInitStructure.TIM_OSSRState = TIM_OSSRState_Enable;
  TIM_BDTRInitStructure.TIM_OSSIState = TIM_OSSIState_Enable;
  TIM_BDTRInitStructure.TIM_LOCKLevel = TIM_LOCKLevel_1;
  TIM_BDTRInitStructure.TIM_DeadTime = PWM_TIM_Deadtime;
  TIM_BDTRInitStructure.TIM_Break = TIM_Break_Enable;
  TIM_BDTRInitStructure.TIM_BreakPolarity = TIM_BreakPolarity_High;
  TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

  TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);
	
	/* TIM1 counter enable */
  TIM_Cmd(PWM_TIM, ENABLE);
	
	/* Main Output Enable */
  TIM_CtrlPWMOutputs(PWM_TIM, ENABLE);
}

void PWM_Init(void) {
	// B?t clock cho GPIOA v� TIM2
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
		GPIO_InitTypeDef GPIO_InitStruct;
	
	// C?u h�nh ch�n PA0 v� PA1 l�m d?u ra PWM
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	// G�n ch?c nang AF cho TIM2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);  // PA0 - TIM2_CH1
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);  // PA1 - TIM2_CH2
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	TIM_OCInitTypeDef TIM_OCStruct;
	
	// C?u h�nh t?n s? co b?n cho TIM2
	TIM_TimeBaseStruct.TIM_Period = 8399;                      // Chu k? (ARR)
	TIM_TimeBaseStruct.TIM_Prescaler = 0 ;                   // Chia t?n s? (84 MHz / (Prescaler + 1))
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;     // Kh�ng chia t?n s? xung nh?p
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up; // �?m l�n
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct);
	
	// C?u h�nh PWM cho k�nh 1 (PA0)
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM1;               // Ch? d? PWM 1
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;   // K�ch ho?t xu?t PWM
	TIM_OCStruct.TIM_Pulse = 0;                            // �? r?ng xung (CCR1)
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_High;       // Xung m?c cao
	TIM_OC1Init(TIM2, &TIM_OCStruct);

	// C?u h�nh PWM cho k�nh 2 (PA1)
	TIM_OCStruct.TIM_Pulse = 0;                            // �? r?ng xung (CCR2)
	TIM_OC2Init(TIM2, &TIM_OCStruct);
	
	// K�ch ho?t ch? d? Preload
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
	
	// K�ch ho?t TIM2
	TIM_ARRPreloadConfig(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

//PID PID_1={30,10,0.01,0,0,0,0}; 
//PID PID_2={30,10,0.01,0,0,0,0,0};

PID PID_1={6,0.09,0.0,0,0,0,0}; 
PID PID_2={6,0.09,0.0,0,0,0,0,0};

void Motor_Control(void)
{
		Motor_1.Output = (Motor_Control_Speed(&PID_1,vel_right,Car.v_right) + 8400)/6; //R
		Motor_2.Output = (Motor_Control_Speed(&PID_2,vel_left,Car.v_left) + 8400)/6;	//L	
	
//	Motor_1.Output = (Motor_Control_Speed(&PID_1,filtered_speed_right,55) + 8400)/6; //R
//	Motor_2.Output = (Motor_Control_Speed(&PID_2,filtered_speed_left,55) + 8400)/6;	//L
	
	if (PID_1.Output ==0)
		Motor_1.Output = 0;
	else if (Motor_1.Output > max)
		Motor_1.Output = max;
	else if (Motor_1.Output < min)
		Motor_1.Output = min;

	if (PID_2.Output == 0)
		Motor_2.Output = 0;
	else if (Motor_2.Output > max)
		Motor_2.Output = max ;
	else if (Motor_2.Output < min)
		Motor_2.Output = min ;	
	
	TIM2->CCR1 = (uint32_t)Motor_1.Output; //R
	TIM2->CCR2 = (uint32_t)Motor_2.Output; //L
	
//	TIM2->CCR1 = 893; //R
//	TIM2->CCR2 = 893; //L
}	

int32_t Motor_Control_Speed(PID* pid, double current,double setpoint)
{
	pid->Error  = setpoint-current;
	pid->P_part = pid->Kp*pid->Error;
	pid->I_part = pid->pre_I_part + 0.5*pid->Ki*T_Sample*(pid->Error + pid->pre_Error);
	pid->D_part = (pid->Kd*( pid->Error - pid->pre_Error))/T_Sample;
	pid->Output = pid->pre_Output + pid->P_part + pid->I_part + pid->D_part ;
	
	pid->pre_I_part = pid->I_part;
	pid->pre_pre_Error = pid->pre_Error;
	pid->pre_Error = pid->Error;
	
	if (pid->Output>duty_max)
		pid->Output=duty_max;	
	if (pid->Output<duty_min)
	pid->Output=duty_min;	
	pid->pre_Output = pid->Output;
	return pid->Output;
}


