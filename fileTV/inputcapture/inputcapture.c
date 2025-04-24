#include "inputcapture.h"

GPIO_InitTypeDef 				INCPGPIO;
NVIC_InitTypeDef 				ngat;
TIM_TimeBaseInitTypeDef INCPTIM;
TIM_ICInitTypeDef 			INCP;

extern int encoder_left,encoder_right;
extern float vel_left,vel_right; // tocdo = left,tocdo_2 = right
extern vel_motor Motor_speed;

void INCP_GPIO_config(void)
{
  // encoder left
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	INCPGPIO.GPIO_Mode=GPIO_Mode_AF;
	INCPGPIO.GPIO_OType=GPIO_OType_PP;
	INCPGPIO.GPIO_Pin=GPIO_Pin_4;
	INCPGPIO.GPIO_PuPd=GPIO_PuPd_UP;
	INCPGPIO.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&INCPGPIO);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);

	INCPGPIO.GPIO_Mode=GPIO_Mode_IN;
	INCPGPIO.GPIO_OType=GPIO_OType_PP;
	INCPGPIO.GPIO_Pin=GPIO_Pin_5;
	INCPGPIO.GPIO_PuPd=GPIO_PuPd_UP;
	INCPGPIO.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&INCPGPIO);

	NVIC_SetPriorityGrouping(2);	
	ngat.NVIC_IRQChannel=TIM3_IRQn;
	ngat.NVIC_IRQChannelCmd=ENABLE;
	ngat.NVIC_IRQChannelPreemptionPriority=0;
	ngat.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&ngat);
	
	//encoder right
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	INCPGPIO.GPIO_Mode=GPIO_Mode_AF;
	INCPGPIO.GPIO_OType=GPIO_OType_PP;
	INCPGPIO.GPIO_Pin=GPIO_Pin_6;
	INCPGPIO.GPIO_PuPd=GPIO_PuPd_UP;
	INCPGPIO.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&INCPGPIO);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_TIM4);

	INCPGPIO.GPIO_Mode=GPIO_Mode_IN;
	INCPGPIO.GPIO_OType=GPIO_OType_PP;
	INCPGPIO.GPIO_Pin=GPIO_Pin_7;
	INCPGPIO.GPIO_PuPd=GPIO_PuPd_UP;
	INCPGPIO.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&INCPGPIO);

	NVIC_SetPriorityGrouping(2);	
	ngat.NVIC_IRQChannel=TIM4_IRQn;
	ngat.NVIC_IRQChannelCmd=ENABLE;
	ngat.NVIC_IRQChannelPreemptionPriority=0;
	ngat.NVIC_IRQChannelSubPriority=2;
	NVIC_Init(&ngat);
}

// config timer 3
void INCP_TIMER_CONFIG()
{
	// encoder left
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

	INCPTIM.TIM_ClockDivision=TIM_CKD_DIV1;
	INCPTIM.TIM_CounterMode=TIM_CounterMode_Up;
	INCPTIM.TIM_Period=499;
	INCPTIM.TIM_Prescaler=8399;
	INCPTIM.TIM_RepetitionCounter=0;

	TIM_TimeBaseInit(TIM3,&INCPTIM);

	//encoder right
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);

	INCPTIM.TIM_ClockDivision=TIM_CKD_DIV1;
	INCPTIM.TIM_CounterMode=TIM_CounterMode_Up;
	INCPTIM.TIM_Period=499;
	INCPTIM.TIM_Prescaler=8399;
	INCPTIM.TIM_RepetitionCounter=0;

	TIM_TimeBaseInit(TIM4,&INCPTIM);
}

// config Input Capture
void INCP_CONFIG(void)
{
	// encoder left
	INCP.TIM_Channel     = TIM_Channel_1;
	INCP.TIM_ICFilter    = 15;
	INCP.TIM_ICPolarity  = TIM_ICPolarity_Rising;
	INCP.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	INCP.TIM_ICSelection = TIM_ICSelection_DirectTI;

	TIM_ICInit(TIM3,&INCP);
		
	TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM3,ENABLE);
	//encoder right
	INCP.TIM_Channel=TIM_Channel_1;
	INCP.TIM_ICFilter=15;
	INCP.TIM_ICPolarity=TIM_ICPolarity_Rising;
	INCP.TIM_ICPrescaler=TIM_ICPSC_DIV1;
	INCP.TIM_ICSelection=TIM_ICSelection_DirectTI;

	TIM_ICInit(TIM4,&INCP);
		
	TIM_ITConfig(TIM4,TIM_IT_CC1,ENABLE);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	TIM_Cmd(TIM4,ENABLE);
}

// interrupt timer 3 encoder left
void TIM3_IRQHandler(){

	if(TIM_GetITStatus(TIM3,TIM_IT_CC1)==1)
	{
		encoder_left++;

		TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);
	}
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==1)
	{
//	  vel_left=(((double)encoder_left*60)/(1000*0.05));
//		encoder_left=0;
		
	  TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	}
}

// interrupt timer 4 encoder right
void TIM4_IRQHandler()
{
	if(TIM_GetITStatus(TIM4,TIM_IT_CC1)==1)
	{
		encoder_right++;
		TIM_ClearITPendingBit(TIM4,TIM_IT_CC1);
	}
	
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==1)
	{
//		vel_right=(((double)encoder_right*60)/(1000*0.05));
//		encoder_right=0;
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
void Get_speed()
{
	  vel_left=(((double)encoder_left*60)/(1000*0.05));
		encoder_left=0;
	
	  vel_right=(((double)encoder_right*60)/(1000*0.05));
		encoder_right=0;
}	