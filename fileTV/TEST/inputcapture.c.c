#include "inputcapture.h"

void INCP_GPIO_config(void)
{RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
GPIO_InitTypeDef INCPGPIO;
INCPGPIO.GPIO_Mode=GPIO_Mode_AF;
INCPGPIO.GPIO_OType=GPIO_OType_PP;
INCPGPIO.GPIO_Pin=GPIO_Pin_6;
INCPGPIO.GPIO_PuPd=GPIO_PuPd_NOPULL;
INCPGPIO.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_Init(GPIOC,&INCPGPIO);
GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM3);

INCPGPIO.GPIO_Mode=GPIO_Mode_IN;
INCPGPIO.GPIO_OType=GPIO_OType_PP;
INCPGPIO.GPIO_Pin=GPIO_Pin_7;
INCPGPIO.GPIO_PuPd=GPIO_PuPd_NOPULL;
INCPGPIO.GPIO_Speed=GPIO_Speed_50MHz;
GPIO_Init(GPIOC,&INCPGPIO);

	
NVIC_InitTypeDef ngat;
ngat.NVIC_IRQChannel=TIM3_IRQn;
ngat.NVIC_IRQChannelCmd=ENABLE;
ngat.NVIC_IRQChannelPreemptionPriority=0;
ngat.NVIC_IRQChannelSubPriority=1;
NVIC_Init(&ngat);
}
void INCP_TIMER_CONFIG()
{TIM_TimeBaseInitTypeDef INCPTIM;
RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);

INCPTIM.TIM_ClockDivision=TIM_CKD_DIV1;
INCPTIM.TIM_CounterMode=TIM_CounterMode_Up;
INCPTIM.TIM_Period=1000;
INCPTIM.TIM_Prescaler=16799;
INCPTIM.TIM_RepetitionCounter=0;

TIM_TimeBaseInit(TIM3,&INCPTIM);

}
void INCP_CONFIG(void)
{
TIM_ICInitTypeDef INCP;
INCP.TIM_Channel=TIM_Channel_1;
INCP.TIM_ICFilter=0;
INCP.TIM_ICPolarity=TIM_ICPolarity_Rising;
INCP.TIM_ICPrescaler=TIM_ICPSC_DIV1;
INCP.TIM_ICSelection=TIM_ICSelection_DirectTI;

TIM_ICInit(TIM3,&INCP);
	
	
TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);

TIM_Cmd(TIM3,ENABLE);
}
  int cout1;
int sovong;
  int cout2;
 double tocdo;
 char huong;
double vitri;
 int dem;
 int gt1;
 int gt2;

void TIM3_IRQHandler()
{

if(TIM_GetITStatus(TIM3,TIM_IT_CC1)==1)
{
cout1++;	
cout2++;
if(cout1==334)
{
cout1=0;
sovong++;
}
vitri=((double)cout1/334)*360;
TIM_ClearITPendingBit(TIM3,TIM_IT_CC1);

}

if(TIM_GetITStatus(TIM3,TIM_IT_Update)==1)
{
tocdo=((double)cout2/334)*10*60;
	cout2=0;
TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
}
}