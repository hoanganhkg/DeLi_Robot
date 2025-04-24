///* Includes */
//#include "MPU6050.h"
////#include "stm32f10x_i2c.h"

///** @defgroup MPU6050_Library
//* @{
//*/

///** Power on and prepare for general usage.
// * This will activate the device and take it out of sleep mode (which must be done
// * after start-up). This function also sets both the accelerometer and the gyroscope
// * to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
// * the clock source to use the X Gyro for reference, which is slightly better than
// * the default internal clock source.
// */
//void MPU6050_Initialize() 
//{
//    MPU6050_SetClockSource(MPU6050_CLOCK_PLL_XGYRO);
//    MPU6050_SetFullScaleGyroRange(MPU6050_GYRO_FS_500);
//    MPU6050_SetFullScaleAccelRange(MPU6050_ACCEL_FS_2);
//    MPU6050_SetSleepModeStatus(DISABLE);
//}

////********************************************************
////! disable bypass for extenal configuration
////! enable master mode
////! (ONLY IN GY-86 )
//void MPU6050_Master_En(void)
//{
//		Enable_MPU6050_I2C_Master();
//	  Setup_Slave0_HMC5883L();
//}

////*****************************************************************************
////!	enable/disable bypass for external configuration
////! disable master mode and can read/write on HMC5883L
////  (ONLY IN GY-86)
////*****************************************************************************
//void MPU6050_Bypass_En(void)
//{
//	MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS,MPU6050_RA_INT_PIN_CFG,MPU6050_INTCFG_I2C_BYPASS_EN_BIT,ENABLE);
//}

//void Enable_MPU6050_I2C_Master()
//{
//	uint8_t mst_en_bit =  0x01 << MPU6050_USERCTRL_I2C_MST_EN_BIT;
//	uint8_t dis_reg_bit = 0x00;
//	uint8_t mst_ctrl_bit = 0x0D;//0x0D
//	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &mst_en_bit, MPU6050_RA_USER_CTRL);      //USER_CTRL     -- DMP_EN=0 ; FIFO_EN=0 ; I2C_MST_EN=1 (I2C master mode) ; I2C_IF_DIS=0 ; FIFO_RESET=0 ; I2C_MST_RESET=0 ; SIG_COND_RESET=0
//	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &dis_reg_bit, MPU6050_RA_INT_PIN_CFG);      //INT_PIN_CFG -- INT_LEVEL=0 ; INT_OPEN=0 ; LATCH_INT_EN=0 ; INT_RD_CLEAR=0 ; FSYNC_INT_LEVEL=0 ; FSYNC_INT_EN=0 ; I2C_BYPASS_EN=0 ; CLKOUT_EN=0
//	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &mst_ctrl_bit, MPU6050_RA_I2C_MST_CTRL);        //I2C_MST_CTRL  -- MULT_MST_EN=0 ; WAIT_FOR_ES=0 ; SLV_3_FIFO_EN=0 ; I2C_MST_P_NSR=0 ; I2C_MST_CLK=13 (I2C slave speed bus = 400kHz)
//}

//void Setup_Slave0_HMC5883L()
//{
//	//Setup AUX I2C slaves
//	uint8_t data[3] = {0x9E,0x03,0x86};
//	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &data[0], MPU6050_RA_I2C_SLV0_ADDR);   //I2C_SLV0_ADDR -- I2C_SLV0_RW=1 (read operation) ; I2C_SLV0_ADDR=0x1E (HMC5883L 7bit address)
//	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &data[1], MPU6050_RA_I2C_SLV0_REG);    //I2C_SLV0_REG  -- 6 data bytes of HMC5883 are stored in register whose first address is 0x03
//	MPU6050_I2C_ByteWrite(MPU6050_DEFAULT_ADDRESS, &data[2], MPU6050_RA_I2C_SLV0_CTRL);   //I2C_SLV0_CTRL -- I2C_SLV0_EN=1 ; I2C_SLV0_BYTE_SW=0 ; I2C_SLV0_REG_DIS=0 ; I2C_SLV0_GRP=0 ; I2C_SLV0_LEN=6 bytes
//}

//void HMC5883L_GetHeading_From_MPU6050(int16_t* Mag)
//{
//	unsigned char BUFFER[6];
//	MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, BUFFER, MPU6050_RA_EXT_SENS_DATA_00, 6);

//	Mag[0] = ((BUFFER[0] * 256 + BUFFER[1]));
//	Mag[1] = ((BUFFER[4] * 256 + BUFFER[5]));
//	Mag[2] = ((BUFFER[2] * 256 + BUFFER[3]));
//}
///** Verify the I2C connection.
// * Make sure the device is connected and responds as expected.
// * @return True if connection is valid, FALSE otherwise
// */
//bool MPU6050_TestConnection() 
//{
//    if(MPU6050_GetDeviceID() == 0x34) //0b110100; 8-bit representation in hex = 0x34
//      return TRUE;
//    else
//      return FALSE;
//}
//// WHO_AM_I register

///** Get Device ID.
// * This register is used to verify the identity of the device (0b110100).
// * @return Device ID (should be 0x68, 104 dec, 150 oct)
// * @see MPU6050_RA_WHO_AM_I
// * @see MPU6050_WHO_AM_I_BIT
// * @see MPU6050_WHO_AM_I_LENGTH
// */
//uint8_t MPU6050_GetDeviceID()
//{
//    uint8_t tmp;
//    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, &tmp);
//    return tmp; 
//}
///** Set clock source setting.
// * An internal 8MHz oscillator, gyroscope based clock, or external sources can
// * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
// * or an external source is chosen as the clock source, the MPU-60X0 can operate
// * in low power modes with the gyroscopes disabled.
// *
// * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
// * However, it is highly recommended that the device be configured to use one of
// * the gyroscopes (or an external clock source) as the clock reference for
// * improved stability. The clock source can be selected according to the following table:
// *
// * <pre>
// * CLK_SEL | Clock Source
// * --------+--------------------------------------
// * 0       | Internal oscillator
// * 1       | PLL with X Gyro reference
// * 2       | PLL with Y Gyro reference
// * 3       | PLL with Z Gyro reference
// * 4       | PLL with external 32.768kHz reference
// * 5       | PLL with external 19.2MHz reference
// * 6       | Reserved
// * 7       | Stops the clock and keeps the timing generator in reset
// * </pre>
// *
// * @param source New clock source setting
// * @see MPU6050_GetClockSource()
// * @see MPU6050_RA_PWR_MGMT_1
// * @see MPU6050_PWR1_CLKSEL_BIT
// * @see MPU6050_PWR1_CLKSEL_LENGTH
// */
//void MPU6050_SetClockSource(uint8_t source) 
//{
//    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
//}

///** Set full-scale gyroscope range.
// * @param range New full-scale gyroscope range value
// * @see MPU6050_GetFullScaleGyroRange()
// * @see MPU6050_GYRO_FS_250
// * @see MPU6050_RA_GYRO_CONFIG
// * @see MPU6050_GCONFIG_FS_SEL_BIT
// * @see MPU6050_GCONFIG_FS_SEL_LENGTH
// */
//void MPU6050_SetFullScaleGyroRange(uint8_t range) 
//{
//    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
//}

//// GYRO_CONFIG register

///** Get full-scale gyroscope range.
// * The FS_SEL parameter allows setting the full-scale range of the gyro sensors,
// * as described in the table below.
// *
// * <pre>
// * 0 = +/- 250 degrees/sec
// * 1 = +/- 500 degrees/sec
// * 2 = +/- 1000 degrees/sec
// * 3 = +/- 2000 degrees/sec
// * </pre>
// *
// * @return Current full-scale gyroscope range setting
// * @see MPU6050_GYRO_FS_250
// * @see MPU6050_RA_GYRO_CONFIG
// * @see MPU6050_GCONFIG_FS_SEL_BIT
// * @see MPU6050_GCONFIG_FS_SEL_LENGTH
// */
//uint8_t MPU6050_GetFullScaleGyroRange() 
//{
//    uint8_t tmp;
//    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, &tmp);
//    return tmp;
//}
///** Get full-scale accelerometer range.
// * The FS_SEL parameter allows setting the full-scale range of the accelerometer
// * sensors, as described in the table below.
// *
// * <pre>
// * 0 = +/- 2g
// * 1 = +/- 4g
// * 2 = +/- 8g
// * 3 = +/- 16g
// * </pre>
// *
// * @return Current full-scale accelerometer range setting
// * @see MPU6050_ACCEL_FS_2
// * @see MPU6050_RA_ACCEL_CONFIG
// * @see MPU6050_ACONFIG_AFS_SEL_BIT
// * @see MPU6050_ACONFIG_AFS_SEL_LENGTH
// */
//uint8_t MPU6050_GetFullScaleAccelRange() 
//{
//    uint8_t tmp;
//    MPU6050_ReadBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, &tmp);
//    return tmp;
//}
///** Set full-scale accelerometer range.
// * @param range New full-scale accelerometer range setting
// * @see MPU6050_GetFullScaleAccelRange()
// */
//void MPU6050_SetFullScaleAccelRange(uint8_t range) 
//{
//    MPU6050_WriteBits(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
//}

///** Get sleep mode status.
// * Setting the SLEEP bit in the register puts the device into very low power
// * sleep mode. In this mode, only the serial interface and internal registers
// * remain active, allowing for a very low standby current. Clearing this bit
// * puts the device back into normal mode. To save power, the individual standby
// * selections for each of the gyros should be used if any gyro axis is not used
// * by the application.
// * @return Current sleep mode enabled status
// * @see MPU6050_RA_PWR_MGMT_1
// * @see MPU6050_PWR1_SLEEP_BIT
// */
//bool MPU6050_GetSleepModeStatus() 
//{
//    uint8_t tmp;
//    MPU6050_ReadBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, &tmp);
//    if(tmp == 0x00)
//      return FALSE;
//    else
//      return TRUE;    
//}
///** Set sleep mode status.
// * @param enabled New sleep mode enabled status
// * @see MPU6050_GetSleepModeStatus()
// * @see MPU6050_RA_PWR_MGMT_1
// * @see MPU6050_PWR1_SLEEP_BIT
// */
//void MPU6050_SetSleepModeStatus(FunctionalState NewState) 
//{
//    MPU6050_WriteBit(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, NewState);
//}

///** Get raw 6-axis motion sensor readings (accel/temp/gyro).
// * Retrieves all currently available motion sensor values.
// * @param AccelGyro 16-bit signed integer array of length 7
// * @see MPU6050_RA_ACCEL_XOUT_H
// * data 0 -> 2 : Accel
// * data 3      : Temp
// * data 4 -> 6 : Gyro
// */
//void MPU6050_GetRawAccelTempGyro(int16_t* AccelGyro) 
//{
//    uint8_t dataR[14],i; 
//    MPU6050_I2C_BufferRead(MPU6050_DEFAULT_ADDRESS, dataR, MPU6050_RA_ACCEL_XOUT_H, 14);

//    for(i=0;i<7;i++) 
//    AccelGyro[i]=(dataR[i*2]<<8)|dataR[i*2+1];   

//}

///** Write multiple bits in an 8-bit device register.
// * @param slaveAddr I2C slave device address
// * @param regAddr Register regAddr to write to
// * @param bitStart First bit position to write (0-7)
// * @param length Number of bits to write (not more than 8)
// * @param data Right-aligned value to write
// */
//void MPU6050_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) 
//{
//    //      010 value to write
//    // 76543210 bit numbers
//    //    xxx   args: bitStart=4, length=3
//    // 00011100 mask byte
//    // 10101111 original value (sample)
//    // 10100011 original & ~mask
//    // 10101011 masked | value
//    uint8_t tmp, mask;
//    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
//    mask = ((1 << length) - 1) << (bitStart - length + 1);
//    data <<= (bitStart - length + 1); // shift data into correct position
//    data &= mask; // zero all non-important bits in data
//    tmp &= ~(mask); // zero all important bits in existing byte
//    tmp |= data; // combine data with existing byte
//    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr);   
//}
///** write a single bit in an 8-bit device register.
// * @param slaveAddr I2C slave device address
// * @param regAddr Register regAddr to write to
// * @param bitNum Bit position to write (0-7)
// * @param value New bit value to write
// */
//void MPU6050_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) 
//{
//    uint8_t tmp;
//    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
//    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
//    MPU6050_I2C_ByteWrite(slaveAddr,&tmp,regAddr); 
//}
///** Read multiple bits from an 8-bit device register.
// * @param slaveAddr I2C slave device address
// * @param regAddr Register regAddr to read from
// * @param bitStart First bit position to read (0-7)
// * @param length Number of bits to read (not more than 8)
// * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
// * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
// */
//void MPU6050_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data) 
//{
//    // 01101001 read byte
//    // 76543210 bit numbers
//    //    xxx   args: bitStart=4, length=3
//    //    010   masked
//    //   -> 010 shifted
//    uint8_t tmp, mask;
//    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1); 
//    mask = ((1 << length) - 1) << (bitStart - length + 1);
//    tmp &= mask;
//    tmp >>= (bitStart - length + 1);
//    *data = tmp;
//}

///** Read a single bit from an 8-bit device register.
// * @param slaveAddr I2C slave device address
// * @param regAddr Register regAddr to read from
// * @param bitNum Bit position to read (0-7)
// * @param data Container for single bit value
// * @param timeout Optional read timeout in milliseconds (0 to disable, leave off to use default class value in readTimeout)
// */
//void MPU6050_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data) 
//{
//    uint8_t tmp;
//    MPU6050_I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);  
//    *data = tmp & (1 << bitNum);
//}

///**
//* @brief  Initializes the I2C peripheral used to drive the MPU6050
//* @param  None
//* @return None
//*/
//void MPU6050_I2C_Init()
//{

//}

///**
//* @brief  Writes one byte to the  MPU6050.
//* @param  slaveAddr : slave address MPU6050_DEFAULT_ADDRESS
//* @param  pBuffer : pointer to the buffer  containing the data to be written to the MPU6050.
//* @param  writeAddr : address of the register in which the data will be written
//* @return None
//*/
//bool MPU6050_I2C_ByteWrite(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t writeAddr)
//{
//	uint32_t time;
//	I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE); 
////  ENTR_CRT_SECTION();

//	/* While the bus is busy */
//	time = MPU6050_I2C_TIMEOUT;
//	while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY))
//	{
//		if((time--) == 0) return false;
//	}
//	
//  /* Send START condition */
//  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

//  /* Test on EV5 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send MPU6050 address for write */
//  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter);

//  /* Test on EV6 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send the MPU6050's internal address to write to */
//  I2C_SendData(MPU6050_I2C, writeAddr);

//  /* Test on EV8 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//	{
//		if((time--) == 0) return false;
//	}

//	/* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
//	__disable_irq();
//	
//  /* Send the byte to be written */
//  I2C_SendData(MPU6050_I2C, *pBuffer);

//  /* Test on EV8 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send STOP condition */
//  I2C_GenerateSTOP(MPU6050_I2C, ENABLE);

//	/* ENABLE ALL INTERRUPT */
//	__enable_irq();
//	return true;
// // EXT_CRT_SECTION();

//}

///**
//* @brief  Reads a block of data from the MPU6050.
//* @param  slaveAddr  : slave address MPU6050_DEFAULT_ADDRESS
//* @param  pBuffer : pointer to the buffer that receives the data read from the MPU6050.
//* @param  readAddr : MPU6050's internal address to read from.
//* @param  NumByteToRead : number of bytes to read from the MPU6050 ( NumByteToRead >1  only for the Mgnetometer readinf).
//* @return None
//*/

//bool MPU6050_I2C_BufferRead(uint8_t slaveAddr, uint8_t* pBuffer, uint8_t readAddr, uint16_t NumByteToRead)
//{
//	uint32_t time;
// // ENTR_CRT_SECTION();

//  /* While the bus is busy */
//	time = MPU6050_I2C_TIMEOUT;
//  while(I2C_GetFlagStatus(MPU6050_I2C, I2C_FLAG_BUSY))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send START condition */
//  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

//  /* Test on EV5 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send MPU6050 address for write */
//  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Transmitter); 

//  /* Test on EV6 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Clear EV6 by setting again the PE bit */
//  I2C_Cmd(MPU6050_I2C, ENABLE);

//  /* Send the MPU6050's internal address to write to */
//  I2C_SendData(MPU6050_I2C, readAddr);

//  /* Test on EV8 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send STRAT condition a second time */
//  I2C_GenerateSTART(MPU6050_I2C, ENABLE);

//  /* Test on EV5 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_MODE_SELECT))
//	{
//		if((time--) == 0) return false;
//	}

//  /* Send MPU6050 address for read */
//  I2C_Send7bitAddress(MPU6050_I2C, slaveAddr, I2C_Direction_Receiver);

//	/* SHOULD DISABLE INTERRUPT HERE, IF NOT IT MAY BE CORRUPT I2C TRANSACTION */
//	__disable_irq();
//	
//  /* Test on EV6 and clear it */
//	time = MPU6050_I2C_TIMEOUT;
//  while(!I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
//	{
//		if((time--) == 0) return false;
//	}

//  /* While there is data to be read */
//  while(NumByteToRead)
//  {
//    if(NumByteToRead == 1)
//    {
//      /* Disable Acknowledgement */
//      I2C_AcknowledgeConfig(MPU6050_I2C, DISABLE);

//      /* Send STOP Condition */
//      I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
//    }

//    /* Test on EV7 and clear it */
//    if(I2C_CheckEvent(MPU6050_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
//    {
//      /* Read a byte from the MPU6050 */
//      *pBuffer = I2C_ReceiveData(MPU6050_I2C);

//      /* Point to the next location where the byte read will be saved */
//      pBuffer++;

//      /* Decrement the read bytes counter */
//      NumByteToRead--;
//    }
//  }

//  /* Wait to make sure that STOP control bit has been cleared */
//	time = MPU6050_I2C_TIMEOUT;
//	while(MPU6050_I2C->CR1 & I2C_CR1_STOP)
//	{
//			if((time--) == 0) return false;
//	}  
//  /* Enable Acknowledgement to be ready for another reception */
//  I2C_AcknowledgeConfig(MPU6050_I2C, ENABLE);

//	/* ENABLE ALL INTERRUPT */
//	__enable_irq();     
//	return true; 
////  EXT_CRT_SECTION();

//}
