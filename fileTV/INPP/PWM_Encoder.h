#ifndef PWM_Encoder	
#define PWM_Encoder

#include "stm32f4xx.h"

#define ENA_Pin_L298 GPIO_Pin_12
#define GPIOENA GPIOE
#define INA_1_Pin_L298 GPIO_Pin_9 //TIM1_CH1
#define GPIOINA1 GPIOE
#define INA_2_Pin_L298 GPIO_Pin_11 //TIM1_CH2
#define GPIOINA2 GPIOE
#define ENB_Pin_L298 GPIO_Pin_15
#define GPIOENB GPIOE
#define INB_1_Pin_L298 GPIO_Pin_13 //TIM1_CH3
#define GPIOINB1 GPIOE
#define INB_2_Pin_L298 GPIO_Pin_14 //TIM1_CH4
#define GPIOINB2 GPIOE
#define EncoderB_Pin GPIO_Pin_7 //TIM3_CH2
#define EncoderA_Pin GPIO_Pin_6 //TIM3_CH1

#define CW (uint8_t)10
#define CCW (uint8_t)11

static uint8_t Motor_Direction;

void PWM_Init(void);
void Encoder_Int(void);
#endif
