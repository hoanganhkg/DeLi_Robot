#ifndef INCAP_H
#define INCAP_H

#include "stm32f4xx.h"

typedef union{
	float velo[2];
	uint8_t vel_data[8];
}vel_motor;

void INCP_GPIO_config(void);
void INCP_TIMER_CONFIG(void);
void INCP_CONFIG(void);
void Get_speed(void);
#endif