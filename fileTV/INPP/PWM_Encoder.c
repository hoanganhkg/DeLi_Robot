#include "PWM_Encoder.h"

void PWM_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	//NVIC_InitTypeDef NVIC_InitStructure;
	
//GPIOE PWM_Pin
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = INA_1_Pin_L298|INA_2_Pin_L298|INB_1_Pin_L298|INB_2_Pin_L298;
	
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

//GPIOC Encoder_Pin
	GPIO_InitStructure.GPIO_Pin = EncoderA_Pin|EncoderB_Pin;
	
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
//GPIOE EN_Pin
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = ENA_Pin_L298|ENB_Pin_L298;
	
	GPIO_Init(GPIOE,&GPIO_InitStructure);
	
	
//Connect Pin to Timer
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);
	GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);
	
//TIM Setup	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	

	
	TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_BaseInitStructure.TIM_Period = 840-1;// 100KHz
	TIM_BaseInitStructure.TIM_Prescaler = 10000-1; 
	TIM_BaseInitStructure.TIM_RepetitionCounter = 0;
		
	TIM_TimeBaseInit(TIM1,&TIM_BaseInitStructure);
//PWM Setup	

	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

//Channel 1 Setup
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC1Init(TIM1,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);
//Channel 2 Setup	
	TIM_OCInitStructure.TIM_Pulse = 0; 
	TIM_OC2Init(TIM1,&TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1,TIM_OCPreload_Enable);
////Channel 3 Setup
//	TIM_OCInitStructure.TIM_Pulse = (840-1)*70/100; //70% duty cycle
//	TIM_OC3Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC3PreloadConfig(TIM1,TIM_OCPreload_Enable);
////Channel 4 Setup
//	TIM_OCInitStructure.TIM_Pulse = (840-1)*70/100; //70% duty cycle
//	TIM_OC4Init(TIM1,&TIM_OCInitStructure);
//	TIM_OC4PreloadConfig(TIM1,TIM_OCPreload_Enable);
	
	TIM_ARRPreloadConfig(TIM1,ENABLE);
	
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	
	
//	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn ;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
	
}



void Encoder_Int(void)
{
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM3->CR1 |= (1<<7)|(1<<1);
	TIM3->PSC = 84 - 1;
	TIM3->ARR = 0xFFFF;
	TIM3->CR1 &= ~(1<<1);
	TIM3->EGR = 1;
	TIM3->CR1 |= (1<<0);
	
	TIM3->CCMR1 |= (1<<0);
	TIM3->CCER |= (1<<0);
	
	
	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//GPIOC Encoder_Pin		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_InitStructure.GPIO_Pin = EncoderA_Pin;
	GPIOC->AFR[0] &= ~(15<<24);
	GPIOC->AFR[0] |= (2<<24);//AF2
	
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = EncoderB_Pin;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
		
//	DMA_InitTypeDef DMA_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	
//	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
//	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ICapture[1];
//	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(TIM3->CCR1));
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;//Directional
//	DMA_InitStructure.DMA_BufferSize = 1;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;//Enable Shift Addr After 1 Circular
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//Disable Shift Addr After 1 Circular
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//16 Bit
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//16 Bit
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//Circulation
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;//Priority
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_Init(DMA1_Stream4, &DMA_InitStructure);
//	DMA_Cmd(DMA1_Stream4, ENABLE); /* DMA2_Stream0 enable */
		
		NVIC_InitTypeDef NVIC_InitStruct;
		
		TIM_ITConfig(TIM3,TIM_IT_CC1,ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    // Enable the Timer 3 interrupt in the NVIC
    NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);
		NVIC_EnableIRQ(TIM3_IRQn);
}


