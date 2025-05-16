#include "stm32f4xx.h"
#include <stdio.h>
#include "string.h"
#include "UGV_Timer.h"
#include "UGV_ENC.h"
#include "UGV_PWM.h"
#include "UGV_CAN.h"
#include "UART.h"
#include "utils.h"
#include "IMU.h"
#include "inputcapture.h"
#include "IMU_RtxQ.h"
#include "RTC.h"
#include "bno055.h"
extern float 	vel_left,vel_right;
uint8_t rxbuff_USART6[BUFF_SIZE_USART6_Rx];
IMU_UART 	IMU_RQ;
vel_motor CAN_Motor_speed;
IMU_send CAN_IMU;
int encoder_left,encoder_right;
EulerAngles angles;
BNO055_Data imu_data;
extern float filtered_speed_left,filtered_speed_right;
float raw_heading,filter_heading;
/**

 * @defgroup Module Pin define
 * @{
 */
		/** 
		* @brief   Timer Pin define 
		*/
		#define		TIMER_MOTOR				 					TIM5 //40ms
		#define		TIMER_MOTOR_CLK							RCC_APB1Periph_TIM5
		#define 	TIMER_MOTOT_CLK_Cmd					RCC_APB1PeriphClockCmd
		#define		TIMER_MOTOR_Prescaler				8399
		#define		TIMER_MOTOR_Period					399
		
		#define		TIMER_TRANSDATA							TIM6 //200ms
		#define		TIMER_TRANSDATA_CLK					RCC_APB1Periph_TIM6
		#define 	TIMER_TRANSDATA_CLK_Cmd			RCC_APB1PeriphClockCmd
		#define		TIMER_TRANSDATA_Prescaler		8399
		#define		TIMER_TRANSDATA_Period			2000-1
		
		#define		TIMER_SAMPLE								TIM8
		#define		TIMER_SAMPLE_CLK						RCC_APB2Periph_TIM8
		#define 	TIMER_SAMPLE_CLK_Cmd				RCC_APB2PeriphClockCmd
		#define		TIMER_SAMPLE_Prescaler			8399
		#define		TIMER_SAMPLE_Period					499
		
		#define		TIMER_CONTROL								TIM7 //200ms - 5 Hz IMU 
		#define		TIMER_CONTROL_CLK						RCC_APB1Periph_TIM7
		#define 	TIMER_CONTROL_CLK_Cmd				RCC_APB1PeriphClockCmd
		#define		TIMER_CONTROL_Prescaler			8399
		#define		TIMER_CONTROL_Period				100-1
		
		#define   CAN_MOTOR_ID                0X01
		#define   CAN_IMU_Yaw_ID              0X02
/**
 * @}
 */
void Timer_Motor(void)
{
		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	  NVIC_InitTypeDef NVIC_InitStruct;

    TIMER_MOTOT_CLK_Cmd(TIMER_MOTOR_CLK, ENABLE);
	/**/
    TIMER_SAMPLE_CLK_Cmd(TIMER_SAMPLE_CLK, ENABLE);
/**/
    TIM_TimeBaseInitStruct.TIM_Prescaler =  TIMER_MOTOR_Prescaler;
    TIM_TimeBaseInitStruct.TIM_Period = TIMER_MOTOR_Period;
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMER_MOTOR, &TIM_TimeBaseInitStruct);
	
	/**/
	    TIM_TimeBaseInit(TIMER_SAMPLE, &TIM_TimeBaseInitStruct);
	/**/
	
    TIM_ITConfig(TIMER_MOTOR, TIM_IT_Update, ENABLE);
	/**/
	    TIM_ITConfig(TIMER_SAMPLE, TIM_IT_Update, ENABLE);
/**/
    TIM_Cmd(TIMER_MOTOR, ENABLE);
		/**/
		    TIM_Cmd(TIMER_SAMPLE, ENABLE);
/**/
		NVIC_SetPriorityGrouping(3);	
    NVIC_InitStruct.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
	/**/
}


void Timer_TransData(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB1PeriphClockCmd(TIMER_TRANSDATA_CLK, ENABLE);

	TIM_TimeBaseInitStruct.TIM_Prescaler =  TIMER_TRANSDATA_Prescaler;
	TIM_TimeBaseInitStruct.TIM_Period = TIMER_TRANSDATA_Period;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER_TRANSDATA, &TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIMER_TRANSDATA, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM6, ENABLE);
	
	NVIC_SetPriorityGrouping(2);
	NVIC_InitStruct.NVIC_IRQChannel = TIM6_DAC_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void Timer_Control(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	TIMER_CONTROL_CLK_Cmd(TIMER_CONTROL_CLK, ENABLE);

	TIM_TimeBaseInitStruct.TIM_Prescaler = TIMER_CONTROL_Prescaler ;
	TIM_TimeBaseInitStruct.TIM_Period = TIMER_CONTROL_Period;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER_CONTROL, &TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIMER_CONTROL, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMER_CONTROL, ENABLE);
	
	NVIC_SetPriorityGrouping(3);	
	NVIC_InitStruct.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority =0;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}
void Timer_Sample(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	TIMER_SAMPLE_CLK_Cmd(TIMER_SAMPLE_CLK, ENABLE);

	TIM_TimeBaseInitStruct.TIM_Prescaler = TIMER_SAMPLE_Prescaler ;
	TIM_TimeBaseInitStruct.TIM_Period = TIMER_SAMPLE_Period;
	TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMER_SAMPLE, &TIM_TimeBaseInitStruct);
	TIM_ITConfig(TIMER_SAMPLE, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMER_SAMPLE, ENABLE);
	
	NVIC_SetPriorityGrouping(3);	
	NVIC_InitStruct.NVIC_IRQChannel = TIM8_UP_TIM13_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority =1;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}	

/*TIMER 5 CONTROL SPEED*/
void TIM5_IRQHandler(void)
{
	if (TIM_GetITStatus(TIMER_MOTOR, TIM_IT_Update))
	{		
		TIM_ClearITPendingBit(TIMER_MOTOR, TIM_IT_Update);
		GPIO_ToggleBits(GPIOD,GPIO_Pin_12); // LED YELLOW
	
//		Get_ENC_speed(&Motor_1,&Motor_2);
		Get_speed();
		filtered_speed_left = KalmanSpeed_Update(&my_kalman_speed, vel_left);
		filtered_speed_right = KalmanSpeed_Update(&my_kalman_speed, vel_right);
		Motor_Control();
		
		IWDG_ReloadCounter();
	}
}
/*TIMER 6 TRANSMIT DATA*/
volatile uint16_t packet_count = 0; // Ð?m s? gói dã g?i

void TIM6_DAC_IRQHandler()
{		
	if (TIM_GetITStatus(TIM6, TIM_IT_Update))
		{
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
			
			GPIO_ToggleBits(GPIOD, GPIO_Pin_15); // (LED Blue)

			CAN_Motor_speed.velo[0] = filtered_speed_left;   // Gi? s? t?c d? d?ng co trái
			CAN_Motor_speed.velo[1] = filtered_speed_right;   // Gi? s? t?c d? d?ng co ph?i
		
			// Gán giá tr? Yaw t? IMU
//			CAN_IMU.IMU_value[0] = IMU_RQ.Yaw;
      CAN_IMU.IMU_value[0] = filter_heading;
			// G?i d? li?u qua CAN
			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte			
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte			
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
//			
//			CAN1_Send(CAN_MOTOR_ID,CAN_Motor_speed.vel_data,8);
//			CAN1_Send(CAN_IMU_Yaw_ID,CAN_IMU.IMU_data_CAN,8); // Ch? g?i 4 byte
			
			IWDG_ReloadCounter();
		}
}

float yaw_offset = -70.6875;
/*TIMER 7 GET IMU*/
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIMER_CONTROL, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIMER_CONTROL, TIM_IT_Update);
		GPIO_ToggleBits(GPIOD, GPIO_Pin_13); // LED ORANGE
		
		raw_heading = BNO055_Get_Heading();
		filter_heading = KalmanAngle_Update(&kalman_yaw, raw_heading);
		IWDG_ReloadCounter();
	}
}
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIMER_SAMPLE, TIM_IT_Update))
	{
		TIM_ClearITPendingBit(TIMER_SAMPLE, TIM_IT_Update);
		GPIO_ToggleBits(GPIOD, GPIO_Pin_14); // LED RED
//		Get_ENC_speed(&Motor_1,&Motor_2);
//		Get_speed();
	}	
}	
void delay_01ms(uint16_t period)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
	TIM12->PSC = (8400)-1;		// clk = SystemCoreClock / 4 / (PSC+1) *2 = 10KHz
	TIM12->ARR = period-1;
	TIM12->CNT = 0;
	TIM12->EGR = 1;		// update registers;

	TIM12->SR  = 0;		// clear overflow flag
	TIM12->CR1 = 1;		// enable Timer6

	while (!TIM12->SR);
	
	TIM12->CR1 = 0;		// stop Timer6
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, DISABLE);
}

