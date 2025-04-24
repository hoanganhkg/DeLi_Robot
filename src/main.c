#include "stm32f4xx.h"                  // Device header
#include "utils.h"
#include "UGV_CAN.h"
#include "UGV_Timer.h"
#include "UGV_PWM.h"
#include "UGV_ENC.h"
#include "inputcapture.h"
#include "UART.h"
#include "stdlib.h"
#include "IMU.h"
#include "IMU_RtxQ.h"
#include "UGV_Watchdog_timer.h"
#include "bno055.h"
extern uint8_t TxBuffer[200];
uint32_t data;
double time,dist;
void LED_Config(void);
extern char RxBuffer[15];
int main(void)
{	
	PWM_Init();
	TIM2->CCR1 = 0;
	TIM2->CCR2 = 0;
	delay_01ms(10000);
	LED_Config();
  INCP_GPIO_config();
	INCP_TIMER_CONFIG();
	INCP_CONFIG();
//	CAN_Config();
	I2C2_Init();
	BNO055_ApplyCalibration(&calib_data);
//	I2C_Configuration();
//	IMU_Initialize();
//	Timer_TransData();
	Timer_Motor();
	Timer_Control();
//	USART6_Rx_DMA();
	IWDG_Config();
	
	while(1)
	{
		
	}
}

void LED_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}