#include "stm32f4xx.h"
#include "UGV_ENC.h"
#include "UGV_PWM.h"

		#define		ENC1_CLK_Cmd		  								 		RCC_AHB1PeriphClockCmd
		#define		ENC1_CLK															RCC_AHB1Periph_GPIOB
		#define		ENC1_PORT															GPIOB
		
		#define 	ENC1_CH1_PORT 					 						 	GPIOC
		#define		ENC1_CH1_PIN													GPIO_Pin_6
		#define 	ENC1_CH1_SOURCE									 			GPIO_PinSource6
		#define		ENC1_TIM_Channel1											TIM_Channel_1
		#define		ENC1_TIM_ICFilter1										15
		
		#define 	ENC1_CH2_PORT 					 						 	GPIOC
		#define		ENC1_CH2_PIN													GPIO_Pin_7
		#define 	ENC1_CH2_SOURCE									 			GPIO_PinSource7
		#define		ENC1_TIM_Channel2											TIM_Channel_2
		#define		ENC1_TIM_ICFilter2										15
		
		#define		ENC1_TIM_AF														GPIO_AF_TIM3
		#define		ENC1_TIM															TIM3
		#define 	ENC1_TIM_CLK													RCC_APB1Periph_TIM3
		#define		ENC1_TIM_CLK_Cmd											RCC_APB1PeriphClockCmd
		#define		ENC1_TIM_Prescaler										8400-1
		#define		ENC1_TIM_Period												10000-1
		#define		ENC1_TIM_ClockDivision								TIM_CKD_DIV1
		
int32_t Output_int, Rotary, Counter,Time, InputCapture, pre_InputCapture;
int8_t CaptureNumber;
double Pulse_per_second, RPM=0;
double SP,PV,Speed_SP=4000,Position_SP=8000,Speed_PV=0,Position_PV=0,temp=0;


void INC_Config(void)
{
		GPIO_InitTypeDef GPIO_InitStructure;
		TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
		TIM_ICInitTypeDef TIM_ICInitStructure;
	//Encoder 1 :PB8 PB9 Timer 4
			ENC1_CLK_Cmd(ENC1_CLK, ENABLE);
		ENC1_TIM_CLK_Cmd(ENC1_TIM_CLK, ENABLE);

		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin = ENC1_CH1_PIN;
		GPIO_Init(ENC1_PORT, &GPIO_InitStructure);
	
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Pin =  ENC1_CH2_PIN;
		GPIO_Init(ENC1_PORT, &GPIO_InitStructure);
	
		GPIO_PinAFConfig(ENC1_CH1_PORT, ENC1_CH1_SOURCE, ENC1_TIM_AF);
	/*TIM BASE CONFIGURATION*/
			TIM_TimeBaseStructure.TIM_Prescaler = ENC1_TIM_Prescaler;
		TIM_TimeBaseStructure.TIM_ClockDivision = ENC1_TIM_ClockDivision; // = 0
		TIM_TimeBaseStructure.TIM_Period = ENC1_TIM_Period ;  // su dung toi da thanh ghi ARR
		TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
		TIM_TimeBaseInit(ENC1_TIM, &TIM_TimeBaseStructure);
					TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
			TIM_ICInitStructure.TIM_ICFilter = 15;
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			TIM_ICInitStructure.TIM_ICPrescaler = 0;
			TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
			TIM_ICInit(TIM3,&TIM_ICInitStructure);
			TIM_Cmd(TIM3,ENABLE);

//		TIM_SetCounter(ENC1_TIM, ENC1_TIM_SetCounter);
		
		TIM_Cmd(ENC1_TIM, ENABLE);

}
void Read_ENC_speed(void)
	{
//		motor1->pulse = TIM_GetCounter(ENC1_TIM);
//		
//		//motor1
//		motor1->delta_pulse =  ENC1_TIM_SetCounter - motor1->pulse;
//		motor1->speed = 60*100*motor1->delta_pulse/(4*ENC_RESOLUTION*60); //convert pulse/T_sample to rpm

//		TIM_SetCounter(ENC1_TIM, ENC1_TIM_SetCounter);
		
if(TIM_GetFlagStatus(TIM3,TIM_FLAG_CC1)== SET)
 { 
		if(TIM_GetFlagStatus(TIM3,TIM_FLAG_CC1)== SET)
		{ 
		TIM_ClearFlag(TIM3,TIM_FLAG_CC1);
			// Determine rotary direction và count number of angle rotated motor
			if (GPIO_ReadInputDataBit(GPIOC,ENC1_CH2_PIN) == RESET)
			{ 
					Counter++;
			}
			else 
			{ 
					Counter--;
			}
			if(Counter >= 334)
			 {
					Rotary++;
					Counter = 0;
			}
			else if(Counter<-334)
			{
					Rotary--;
					Counter = 0;
			}
InputCapture = TIM_GetCapture1(TIM3); 
		 Time = InputCapture - pre_InputCapture + CaptureNumber*10000;
		 CaptureNumber = 0;
		 pre_InputCapture = InputCapture;
		 if(Time!=0)
		 {
				Pulse_per_second = 10000/Time;
			}
			else Pulse_per_second = 0; 
			// Resolution of encoder: 334(ppr)
		 Speed_PV = Pulse_per_second*60/334; 
		 Position_PV = (Rotary + (double)Counter/334)*360;
		 if(TIM_GetFlagStatus(TIM3,TIM_FLAG_Update) != SET)
		 {
				CaptureNumber++;
		 }
		}
	}


	}
 