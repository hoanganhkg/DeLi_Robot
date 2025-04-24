//#include "stm32f4xx.h"
//#include "IMU.h"
//#include "utils.h"
//#include "stdio.h"
//#include <stdio.h>
//#include "delay_01ms.h"
///******************************************************************************
// *                                                                            *
// *  												VARIABLES	                                        *
// *  														                                              *
// ******************************************************************************/
//#define PI										3.141592
//IMU_t IMU;
//HMC5883L_t HMC5883L;
//double mX = 0,mY = 0, mZ=0;
//double mX_max=0, mX_min=0, mY_max=0, mY_min=0, mX_offset=0, mY_offset=0;
//float i;
//#define mpu6050Addr 0xD0
//uint8_t i2cBuf[8];
//int16_t i2cBuffer[7]={0};
//float accCoef = 0.1f;
//float gyroCoef = 0.9f;
//float gyroXoffset = 0.0f;
//float	gyroYoffset = 0.0f;
//float gyroZoffset = 0.0f;
///******************************************************************************
// *                                                                            *
// *  												PROGRAM	                                          *
// *  														                                              *
// ******************************************************************************/
//void I2C_Configuration(void)
//{
//	I2C_InitTypeDef				I2C_InitStructure;
//	GPIO_InitTypeDef			GPIO_InitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
//	I2C_DeInit(I2C2);
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_ClockSpeed = 100000;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
//	I2C_Init(I2C2, &I2C_InitStructure);
//	I2C_Cmd(I2C2, ENABLE);
//	//B10: SCL
//	//B11: SDA

//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
//	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
//	}
//void IMU_Initialize(void)
//{
//	MPU6050_Initialize();
//	MPU6050_Bypass_En();// ONLY GY-86
//	MPU6050_SetFullScaleGyroRange(0);
//	MPU6050_SetFullScaleAccelRange(0); 
//	HMC5883L_Initialize();
//}

//void MPU6050_Init(void)
//{
//	/*ACtive mpu*/
//	i2cBuf[0] = 107;
//	i2cBuf[1] = 0x00;
//	i2c_master_transmit(I2C2, mpu6050Addr, i2cBuf, 2);
//	
//	/*set accel range = +-2g*/
//	i2cBuf[0] = 28;
//	i2cBuf[1] = 0x00;
//	i2c_master_transmit(I2C2, mpu6050Addr, i2cBuf, 2);
//	
//	/*set GRYo range = +-250deg/s*/
//	i2cBuf[0] = 27;
//	i2cBuf[1] = 0x00;
//	i2c_master_transmit(I2C2, mpu6050Addr, i2cBuf, 2);
//	
//	/*request to read from mpu*/
//	i2cBuf[0] = 28;
//	i2c_master_transmit(I2C2, mpu6050Addr, i2cBuf, 1);
//	i2cBuf[1] = 0x00;
//	i2c_master_recieve(I2C2, mpu6050Addr|0x01, &i2cBuf[1], 1);
//	
//}
//void i2c_master_transmit(I2C_TypeDef* i2cx, uint16_t SlaveAddress, uint8_t *pval, uint16_t length)
//{
//	I2C_GenerateSTART(i2cx, ENABLE);
//	while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_MODE_SELECT));
//	
//	I2C_Send7bitAddress(i2cx, SlaveAddress, I2C_Direction_Transmitter);
//	while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	
//	while(length)
//	{
//		I2C_SendData(i2cx, *pval);
//		while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//		pval++;
//		length--;
//	}
//	
//	I2C_GenerateSTOP(i2cx, ENABLE);
//}

//void i2c_master_recieve(I2C_TypeDef* i2cx, uint16_t SlaveAddress, uint8_t *pval, uint16_t length)
//{
//	uint8_t temp;
//	while(I2C_GetFlagStatus(i2cx, I2C_FLAG_BUSY));
//	
//	I2C_GenerateSTART(i2cx, ENABLE);
//	while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_MODE_SELECT));
//	
//	I2C_Send7bitAddress(i2cx, SlaveAddress, I2C_Direction_Transmitter);
//	while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	
//	I2C_Cmd(i2cx, ENABLE);
//	
//	I2C_GenerateSTART(i2cx, ENABLE);
//	while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_MODE_SELECT));
//	
//	I2C_Send7bitAddress(i2cx, SlaveAddress, I2C_Direction_Receiver);
//	while(!I2C_CheckEvent(i2cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//	
//	if (length > 1)
//    I2C_AcknowledgeConfig(I2C2, ENABLE);
//	while(length)
//	{
//		if (length == 1)
//			I2C_AcknowledgeConfig(I2C2, DISABLE);

//		while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
//    
//		temp = I2C_ReceiveData(I2C2);
//		*pval++ = temp;
//		--length;
//	}
//	
//	I2C_GenerateSTOP(I2C2, ENABLE);
//}
//void getRaw_Accel_Gyro(void)
//{
//	MPU6050_GetRawAccelTempGyro(i2cBuffer);
//	IMU.aX=i2cBuffer[0]/16384.0;
//	IMU.aY=i2cBuffer[1]/16384.0;
//	IMU.aZ=i2cBuffer[2]/16384.0;
//	IMU.gX=i2cBuffer[4]/131.0;
//	IMU.gY=i2cBuffer[5]/131.0;
//	IMU.gZ=i2cBuffer[6]/131.0;
//}
//void getAngle(void)
//{	
//	IMU.Angle_aX = atan2(IMU.aY, sqrt(IMU.aX*IMU.aX + IMU.aZ*IMU.aZ))*180.0/PI;
//	//IMU.Angle_aY= (90.0/100.0)*atan2(-IMU.aX, IMU.aZ)*180.0/PI;
//	IMU.Angle_aY = atan2(IMU.aX, sqrt(IMU.aY*IMU.aY + IMU.aZ*IMU.aZ))*180.0/PI;
//	IMU.gX -= gyroXoffset;
//	IMU.gY -= gyroYoffset;
//	IMU.gZ -= gyroZoffset;
//	IMU.Angle_gX += IMU.gX * 0.01;
//	IMU.Angle_gY += IMU.gY * 0.01;
//	IMU.Angle_gZ += IMU.gZ * 0.01;	
//	IMU.angleX = (gyroCoef * (IMU.angleX + IMU.gX * 0.01)) + (accCoef * IMU.Angle_aX);
//	IMU.angleY = (gyroCoef * (IMU.angleY + IMU.gY * 0.01)) + (accCoef * IMU.Angle_aY);
//	IMU.angleZ = IMU.Angle_gZ;;		
//	IMU.Roll=IMU.angleX;
//	IMU.Pitch=IMU.angleY;
//}

//void kalman_init(IMU_Kalman * p_kalman) {

//    /* We will set the variables like so, these can also be tuned by the user */
//    
//    p_kalman->Q_angle = 0.001f;
//    p_kalman->Q_bias = 0.003f;
//    p_kalman->R_measure = 0.03f;

//    p_kalman->angle = 0.0f; // Reset the angle
//    p_kalman->bias = 0.0f; // Reset bias

//    p_kalman->P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en->wikipedia->org/wiki/Kalman_filter#Example_application->2C_technical
//    p_kalman->P[0][1] = 0.0f;
//    p_kalman->P[1][0] = 0.0f;
//    p_kalman->P[1][1] = 0.0f;
//	
//}
//float kalman_get_angle(IMU_Kalman * p_kalman, float newAngle, float newRate, float dt)
//	{
//		float S, K[2], y, P00_temp, P01_temp;
//    p_kalman->rate = newRate - p_kalman->bias;
//    p_kalman->angle += dt * p_kalman->rate;

//    // Update estimation error covariance - Project the error covariance ahead
//    
//    p_kalman->P[0][0] += dt * (dt*p_kalman->P[1][1] - p_kalman->P[0][1] - p_kalman->P[1][0] + p_kalman->Q_angle);
//    p_kalman->P[0][1] -= dt * p_kalman->P[1][1];
//    p_kalman->P[1][0] -= dt * p_kalman->P[1][1];
//    p_kalman->P[1][1] += p_kalman->Q_bias * dt;

//    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
//    // Calculate Kalman gain - Compute the Kalman gain
//    
//    S = p_kalman->P[0][0] + p_kalman->R_measure; // Estimate error
//    
//    // Kalman gain - This is a 2x1 vector
//    K[0] = p_kalman->P[0][0] / S;
//    K[1] = p_kalman->P[1][0] / S;

//    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
//    
//    y = newAngle - p_kalman->angle; // Angle difference
//    
//    p_kalman->angle += K[0] * y;
//    p_kalman->bias += K[1] * y;

//    // Calculate estimation error covariance - Update the error covariance
//    
//    P00_temp = p_kalman->P[0][0];
//    P01_temp = p_kalman->P[0][1];

//    p_kalman->P[0][0] -= K[0] * P00_temp;
//    p_kalman->P[0][1] -= K[0] * P01_temp;
//    p_kalman->P[1][0] -= K[1] * P00_temp;
//    p_kalman->P[1][1] -= K[1] * P01_temp;
//    return p_kalman->angle;
//}

//void setAngle(IMU_Kalman * p_kalman , float angle) {
//p_kalman->angle=angle;
//}

//// Out heading maganate

//double IMU_Heading(void)
//{
//	int16_t mag[3] = {0};
//	double heading = 0;
//	HMC5883L_GetHeading(mag);
//	if(mag[0]>mX_max) mX_max=mag[0];
//	if(mag[0]<mX_min) mX_min=mag[0];
//	if(mag[2]>mY_max) mY_max=mag[2];
//	if(mag[2]<mY_min) mY_min=mag[2];
//	
//	mX_offset=-(mX_max+mX_min)/2;
//	mY_offset=-(mY_max+mY_min)/2;
//	//HMC5883L.mX = mag[0]+165.5;
//	//HMC5883L.mY = mag[2]+43.78;
//	//HMC5883L.mX=mag[0]+82.5;
//	//HMC5883L.mY=mag[2]-51.5;
//	HMC5883L.mX=mag[0]+mX_offset;
//	HMC5883L.mY=mag[2]+mY_offset;
//	HMC5883L.mZ= mag[1];
//	mX = (HMC5883L.mX*cos(IMU.Pitch*3.1416/180)+HMC5883L.mY*sin(IMU.Roll*3.1416/180)*sin(IMU.Pitch*3.1416/180)+HMC5883L.mZ*cos(IMU.Roll*3.1416/180)*sin(IMU.Pitch*3.1416/180))/1090.0;	//HMC5883L_GAIN = 1090  
//	mY = (HMC5883L.mY*cos(IMU.Roll*3.1416/180)-HMC5883L.mZ*sin(IMU.Roll*3.1416/180))/1090.0;	//	[Gauss]            

//	heading = atan2f(-mY,mX)*180.0/3.1416;
//	//if(heading >180) heading=he; // Revere Clock
//	
//	return -heading;
//}
