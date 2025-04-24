#include "UART.h"
#include "UGV_PWM.h"
#include "UGV_ENC.h"
#include "utils.h"
#include "IMU.h"
#include "math.h"
#include <stdio.h>
char *f;

extern uint8_t TxBuffer[200];

uint8_t RxBuffer[15];
uint8_t velo_left[8];
uint8_t velo_right[8];

 void UARTini()
 { USART_InitTypeDef USART_InitStructure;
	 DMA_InitTypeDef  DMA_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	 /*D5:Tx
	 D6: Rx
	 
	 */
	   /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
   /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_BufferSize = 15 ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)(&(USART3->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;

  /* Configure TX DMA */
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
//  DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t) &TxBuffer;
//  DMA_Init(DMA1_Stream3,&DMA_InitStructure);

  /* Configure RX DMA */
   DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
   DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
   DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t) &RxBuffer ; 
  DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	
	 /* Configure USART Tx and Rx as alternate function push-pull */
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
   GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
   GPIO_InitStructure.GPIO_Pin =GPIO_Pin_8|GPIO_Pin_9;
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	 //USART_OverSampling8Cmd(USART2,DISABLE);
	USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART3, &USART_InitStructure);
	
	
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
  GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
		 USART_DMACmd(USART3,USART_DMAReq_Rx, ENABLE);//|

//	 USART_DMACmd(USART3,USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);//|

//DMA_ITConfig (DMA1_Stream3,DMA_IT_TC,ENABLE);
//    DMA_ITConfig (DMA1_Stream5,DMA_IT_TC,ENABLE);
	 DMA_Cmd(DMA1_Stream3,ENABLE);
	 DMA_Cmd(DMA1_Stream1,ENABLE);
	 
//	 USART_ITConfig(USART2,USART_IT_RXNE,ENABLE); 
//	 USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);
	  /* Enable USART */
  USART_Cmd(USART3, ENABLE);
	
//			 NVIC_InitTypeDef NVIC_InitStruct;
//    NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream1_IRQn;
//    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStruct);

//  NVIC_InitTypeDef NVIC_InitStructure;

//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	/* Enable the USARTx Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);

	

//DMA_InitTypeDef DMA_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	/* GPIOA,USART2,DMA Peripheral clock enable */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
//	
//	/* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
//  
//	/* Configure DMA Initialization Structure */
//	DMA_InitStructure.DMA_BufferSize = BufferSize ;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART2->DR)) ;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	/* Configure TX DMA */
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)&trans_usart ;
//	DMA_Init(DMA1_Stream6,&DMA_InitStructure);
//	/* Configure RX DMA */
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
//	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)&receive_usart ; 
//	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
//	
//	/* Configure PORTA in output pushpull mode */
//  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOA, &GPIO_InitStruct);	

////	RCC->AHB1ENR |= (1<<0);
////	
////	GPIOA->MODER &= ~((1<<4)|(1<<6));
////	GPIOA->MODER |= (1<<5)|(1<<7);
////	
////	GPIOA->PUPDR |= (1<<4);
////	GPIOA->OSPEEDR |= (3<<4);
////	GPIOA->AFR[0] &= ~(15<<8)|(15<<12);
////	GPIOA->AFR[0] |= (7<<8)|(7<<12);
//	
//	/* Configure USART3 Tx/Rx */
//	USART_InitStructure.USART_BaudRate = 9600;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART2, &USART_InitStructure);
//	
//	/* Connect USART pins to AF7 */
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
//	/* Enable DMA USART RX Stream */
//	DMA_Cmd(DMA1_Stream5,ENABLE);
///* Enable DMA USART TX Stream */
////	DMA_Cmd(DMA1_Stream6,ENABLE);
//	USART_DMACmd(USART2,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
//	/* Enable USART */
//	USART_Cmd(USART2, ENABLE);
//	
//	
////	NVIC_InitTypeDef NVIC_InitStructure;

////	/* Enable the USARTx Interrupt */
////	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init(&NVIC_InitStructure);
//
	 
	 

  


  



	


     



}




//char receive_usart[BufferSize];
//char trans_usart[50]={'A'};
//void USART2_Init(void)
//{
//	DMA_InitTypeDef DMA_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStruct;
//	
//	/* GPIOA,USART2,DMA Peripheral clock enable */
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
//	
//	/* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
//  
//	/* Configure DMA Initialization Structure */
//	DMA_InitStructure.DMA_BufferSize = BufferSize ;
//	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
//	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
//	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART2->DR)) ;
//	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//	/* Configure TX DMA */
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)&trans_usart ;
//	DMA_Init(DMA1_Stream6,&DMA_InitStructure);
//	/* Configure RX DMA */
//	DMA_InitStructure.DMA_Channel = DMA_Channel_4 ;
//	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
//	DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)&receive_usart ; 
//	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
//	
//	/* Configure PORTA in output pushpull mode */
//  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
//  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_Init(GPIOA, &GPIO_InitStruct);	

////	RCC->AHB1ENR |= (1<<0);
////	
////	GPIOA->MODER &= ~((1<<4)|(1<<6));
////	GPIOA->MODER |= (1<<5)|(1<<7);
////	
////	GPIOA->PUPDR |= (1<<4);
////	GPIOA->OSPEEDR |= (3<<4);
////	GPIOA->AFR[0] &= ~(15<<8)|(15<<12);
////	GPIOA->AFR[0] |= (7<<8)|(7<<12);
//	
//	/* Configure USART3 Tx/Rx */
//	USART_InitStructure.USART_BaudRate = 9600;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//	USART_Init(USART2, &USART_InitStructure);
//	
//	/* Connect USART pins to AF7 */
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
//  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
//	/* Enable DMA USART RX Stream */
//	DMA_Cmd(DMA1_Stream5,ENABLE);
///* Enable DMA USART TX Stream */
////	DMA_Cmd(DMA1_Stream6,ENABLE);
//	USART_DMACmd(USART2,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE);
//	/* Enable USART */
//	USART_Cmd(USART2, ENABLE);
//	
//	
////	NVIC_InitTypeDef NVIC_InitStructure;

////	/* Enable the USARTx Interrupt */
////	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////	NVIC_Init(&NVIC_InitStructure);
//}
//void send_data(uint8_t data)
//{
//	while(!USART_GetFlagStatus(USART2,USART_FLAG_TXE))
//		USART_SendData(USART2,data);
//	while(!USART_GetFlagStatus(USART2,USART_FLAG_TC));
//}
//void send_char(char *data)
//{
//	while(*data)	send_data(*data++);
//}
//uint8_t receive_data(void)
//{
//	while(!USART_GetFlagStatus(USART2,USART_FLAG_RXNE));
//	return USART_ReceiveData(USART2);
//}
//uint8_t check_password(char password[],char receive_usart[] )
//{
//	for(uint32_t i=0;i<BufferSize;i++){
//		if(password[i]!=receive_usart[i])	return 0;
//	}
//	return 1;
//}
//void reset_buffer(char receive_usart[])
//{
//	for(uint32_t i=0;i<BufferSize;i++){
//		receive_usart[i]=0;
//	}
//}
//void UARTcog(){GPIO_InitTypeDef GPIO_InitStruc;
//RCC->AHB1ENR |=(1<<0);
//	RCC->AHB1ENR |=(1<<17);
//	GPIOA->MODER |=(2<<6)|(2<<4);
//	GPIOA->OSPEEDR |=(3<<4)|(3<<6);
//	GPIOA->AFR[0] |=(7<<8)|(7<<12);
//	USART2->CR1 =0;
//		USART2->CR1|=(1<<13);
//		USART2->CR1|=(1<<2)|(1<<3);
//	 	USART2->CR1 &=~(1<<15);
//		USART2->CR1 &=~(1<<12);
//USART2->BRR	|=(7<<4)|(10<<0);
//}
//void gui(uint8_t c)
//{
//while(!((USART2->SR &(1<<7))));
//USART2->DR=(c);
//while(!((USART2->SR &(1<<6))));
//}
//void guichu(char *text)
//{while(*text) gui(*text++);
//}
//char nhanchu(void){
//	uint8_t chu;
//	while(!(USART1->SR &(1<<5)));
//	chu=USART2->DR;
//	return USART2->DR;
//}
 
//void xulymangdulieu(uint32_t *diachimang,int xoa)
//{
//for(int n=xoa;n>0;n--)
//{
//diachimang++;
//}
//for(int n=17-xoa;n>0;n--)
//{
//*diachimang++=0;
//}
//}
//void Read_Uart(void)
//{

//}

char USART3_Write (char ch)
{
	while(!(USART3-> SR & (1<<7))){}
		USART3-> DR = (ch & 0xff);
		return ch;
	
}

void Send_Char( char *text)
{
	while(*text) USART3_Write(*text++);
	
}
void Get_velo(void)
{
	String_Split(RxBuffer,',',0,velo_left);
	String_Split(RxBuffer,',',1,velo_right);
	
	Car.v_left=Str2Double(velo_left);
	Car.v_right=Str2Double(velo_right);

}

