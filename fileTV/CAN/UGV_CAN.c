#include "stm32f4xx.h"
#include <string.h>
#include "utils.h"
#include "UGV_CAN.h"
#include "UGV_ENC.h"
#include "IMU.h"
#include "IMU_RtxQ.h"
#include "UGV_PWM.h"

CanRxMsg CAN1_RxMessage;
uint8_t IMU_data[8];
uint8_t Velo_data[8];
float Kp1,Ki1,Kd1,Kp2,Ki2,Kd2;
void CAN_Config(void) {
	
	GPIO_InitTypeDef       GPIO_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	NVIC_InitTypeDef       NVIC_InitStructure;

  /* CAN GPIOs configuration **************************************************/

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(CAN_GPIO_CLK, ENABLE);

  /* Connect CAN pins to AF9 */
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
  GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT); 
  
  /* Configure CAN RX and TX pins */
  GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
  GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

  /* CAN configuration ********************************************************/  
  /* Enable CAN clock */
  RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);
  
  /* CAN register init */
  CAN_DeInit(CANx);

  /* CAN cell init */
  CAN_InitStructure.CAN_TTCM = DISABLE;
  CAN_InitStructure.CAN_ABOM = DISABLE;
  CAN_InitStructure.CAN_AWUM = ENABLE;
  CAN_InitStructure.CAN_NART = DISABLE;
  CAN_InitStructure.CAN_RFLM = DISABLE;
  CAN_InitStructure.CAN_TXFP = DISABLE;
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;
    
  /* CAN Baudrate = 1 Mps (CAN clocked at 84 MHz) */
  CAN_InitStructure.CAN_BS1 = CAN_BS1_11tq;
  CAN_InitStructure.CAN_BS2 = CAN_BS2_2tq;
  CAN_InitStructure.CAN_Prescaler = 3;
  CAN_Init(CANx, &CAN_InitStructure);
	
	CAN1->MCR &= ~CAN_MCR_INRQ;  // Clear Initialization Request bit
	uint32_t timeout = 1000000; // Timeout d? tránh vòng l?p vô h?n
		while ((CAN1->MSR & CAN_MSR_INAK) != 0)
		{
				if (--timeout == 0)
				{
						
				}
		}
  /* CAN filter init */
  CAN_FilterInitStructure.CAN_FilterNumber = 0;
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x00<<5;
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x00<<5;
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
  CAN_FilterInit(&CAN_FilterInitStructure);
    
  /* Enable FIFO 0 message pending Interrupt */
  CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
	
	NVIC_SetPriorityGrouping(2);
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
uint32_t timeout = 0;
void CAN1_Send(int _IDstd,uint8_t *data, uint8_t _length)
{
	  CanTxMsg TxMessage;
    uint8_t TransmitMailbox;
    

    memset(&TxMessage, 0, sizeof(CanTxMsg)); // Xóa struct tru?c khi dùng

    TxMessage.StdId = _IDstd; 
    TxMessage.ExtId = 0x00;  
    TxMessage.RTR = CAN_RTR_DATA;
    TxMessage.IDE = CAN_ID_STD;
    TxMessage.DLC = _length; 

    memcpy(TxMessage.Data, data, _length); // Copy d? li?u vào khung CAN

    TransmitMailbox = CAN_Transmit(CAN1, &TxMessage);

    // Ki?m tra tr?ng thái g?i
    while ((CAN_TransmitStatus(CAN1, TransmitMailbox) != CAN_TxStatus_Ok) && (timeout++));
}

void CAN1_RX0_IRQHandler(void)
{
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
	{
		CAN_Receive(CAN1, CAN_FIFO0, &CAN1_RxMessage);
		switch(CAN1_RxMessage.StdId)
		{
			case 0x001: memcpy(&Car.v_left,CAN1_RxMessage.Data,4); // motor speed
									memcpy(&Car.v_right,CAN1_RxMessage.Data+4,4); 
      break;		
      case 0x003: memcpy(&Kp1,CAN1_RxMessage.Data,4); // Kp1
									memcpy(&Kp2,CAN1_RxMessage.Data+4,4); // Kp2
      break;			
			case 0x005: memcpy(&Ki1,CAN1_RxMessage.Data,4); // Ki1
									memcpy(&Ki2,CAN1_RxMessage.Data+4,4); // Ki2
			break;
			case 0x004: memcpy(&Kd1,CAN1_RxMessage.Data,4); // Kd1
									memcpy(&Kd2,CAN1_RxMessage.Data+4,4); // Kd2
			break;
		}
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
	}
}

void CAN_send_package(IMU_send *IMU_CAN)
{
	for(int i=0;i<4;i++)
			{
				CAN1_Send (0x002,&IMU_CAN->IMU_data_CAN[i*8],8);
			}
}	


