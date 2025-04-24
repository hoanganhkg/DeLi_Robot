/******************************************************************************
 *                                                                            *
 *  @Project    MotorControl				                                          *
 *	@File       IMU.h     	                                                  *
 * 	@Author	    Than Trong Khanh Nhat                                         *
 *  @Version 		V1.0	                                                       	*
 *                                                                            *
 ******************************************************************************/
#ifndef IMU_H_
#define IMU_H_

#include "stdint.h"
#include "MPU6050.h"
#include "HMC5883L.h"
#include "math.h"
#include "stdlib.h"
#include "utils.h"

typedef struct
{
	int16_t mX;	//X axis
	int16_t mY;	//Y axis
	int16_t mZ;	//Z axis
}HMC5883L_t;	//Digital compass variables

typedef struct
{
	float gX;
	float gY;
	float gZ;
	float aX;
	float aY;
	float aZ;
	float mX;
	float mY;
	float mZ;

	float angleX;
	float angleY;
	float angleZ;

	float Angle_aX;
	float Angle_aY;
	float Angle_aZ;
	float Angle_gX;
	float Angle_gY;
	float Angle_gZ;
	
	float Alpha;
	float  Roll;
	float	Pitch;
	float  Yaw;
}IMU_t;

typedef struct
{
	double X[2];					  //X^[k]
	double X_previous[2];		//X^[k-1]
	double X_dot[2];				//X*[k]
	double Y[2];
	double P[2][2];				//P[k]
	double P_previous[2][2];		//P[k-1]
	double P_dot[2][2];			//P*[k]
	double K[2];				//K[k]
	double Q[2][2];				//Q
	double R;					//R
	float vantoc;
	float giatoc;
}Kalman_t;
typedef struct {
float Q_angle; // Process noise variance for the accelerometer
float Q_bias; // Process noise variance for the gyro bias
float R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

float angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector

float bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate
float P[2][2]; // Error covariance matrix - This is a 2x2 matrix
} IMU_Kalman;

extern IMU_t IMU;

void I2C_Configuration(void);
void MPU6050_Init(void);
void getRaw_Accel_Gyro(void);
void getAngle(void);
void i2c_master_transmit(I2C_TypeDef* i2cx, uint16_t SlaveAddress, uint8_t *pval, uint16_t length);
void i2c_master_recieve(I2C_TypeDef* i2cx, uint16_t SlaveAddress, uint8_t *pval, uint16_t length);
void kalman_init(IMU_Kalman * p_kalman);
float kalman_get_angle(IMU_Kalman * p_kalman,float newAngle, float newRate, float dt);
void setAngle(IMU_Kalman * p_kalman, float angle);
void IMU_Initialize(void);
double IMU_Heading(void);
#endif
