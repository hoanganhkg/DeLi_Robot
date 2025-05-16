#ifndef INCAP_H
#define INCAP_H

#include "stm32f4xx.h"

typedef union{
	float velo[2];
	uint8_t vel_data[8];
}vel_motor;

typedef struct {
    float speed;         // Giá tr? t?c d? dã l?c
    float P;             // Sai s? u?c lu?ng (error covariance)
    float Q;             // Nhi?u quá trình (process noise)
    float R;             // Nhi?u do lu?ng (measurement noise)
    float K;             // Kalman gain
} KalmanSpeed;

void INCP_GPIO_config(void);
void INCP_TIMER_CONFIG(void);
void INCP_CONFIG(void);
void Get_speed(void);
void KalmanSpeed_Init(KalmanSpeed *kf, float Q, float R) ;
float KalmanSpeed_Update(KalmanSpeed *kf, float measured_speed);
extern KalmanSpeed my_kalman_speed;
#endif