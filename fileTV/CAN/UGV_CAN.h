/********************************************************************************
 *               		                                                            *
 *  Project    		UGV_ARM				                                          			*
 *	File      		UGV_CAN.h     	                                              *
 * 	Author	    	Le Phuc Cong Thanh		                                        *
 *  Created on: 	Feb 26, 2021	                                                *
 *  Version 			V1.0	                                                       	*
 *                                                                   		        *
 ********************************************************************************/
#ifndef UGV_CAN_H
#define UGV_CAN_H
#include "stm32f4xx.h"
#include "IMU_RtxQ.h"
/******************************************************************************
 *                                                                            *
 *  												DEFINE  	                                        *
 *  														                                              *
 ******************************************************************************/

#define label_veloc 0x001

/******************************************************************************
 *                                                                            *
 *  												TYPEDEFINE                                        *
 *  														                                              *
 ******************************************************************************/

typedef union{
	float mf[2];	
	double md;
	uint8_t mb[8];
} mCANdata;

typedef enum{
	sts_new_data,
	sts_no_flag,
}mDatastatus;


/******************************************************************************
 *                                                                            *
 *  												FUNCTION	                                        *
 *  														                                              *
 ******************************************************************************/
 

void CAN_Config(void);
void CAN_SendMessage(uint32_t id, uint8_t *data, uint8_t len);
void CAN_ReceiveMessage();
void CAN1_Send(int _IDstd,uint8_t * data, uint8_t _length);
void CAN_send_package(IMU_send *IMU_CAN);


  #define CANx                       CAN1
  #define CAN_CLK                    RCC_APB1Periph_CAN1
  #define CAN_RX_PIN                 GPIO_Pin_0
  #define CAN_TX_PIN                 GPIO_Pin_1	
  #define CAN_GPIO_PORT              GPIOD
  #define CAN_GPIO_CLK               RCC_AHB1Periph_GPIOD
  #define CAN_AF_PORT                GPIO_AF_CAN1
  #define CAN_RX_SOURCE              GPIO_PinSource0
  #define CAN_TX_SOURCE              GPIO_PinSource1 
#endif