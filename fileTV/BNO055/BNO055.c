#include "bno055.h"

BNO055_Offsets_t calib_data = {22,-18,-35,186,161,-337,-2,1,-1,1000,376};

void I2C2_Init(void) {
    // Enable GPIOB and I2C2 clocks
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    
    // Configure I2C pins (PB10=SCL, PB11=SDA)
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // Open drain for I2C
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Connect I2C2 pins to AF4
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);
    
    // Configure I2C2
    I2C_InitTypeDef I2C_InitStruct;
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00; // STM32 address (not used as master)
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = 100000; // 100 kHz - standard I2C speed
    
    // Initialize I2C2
    I2C_Init(I2C2, &I2C_InitStruct);
    
    // Enable I2C2
    I2C_Cmd(I2C2, ENABLE);
}

void delay_ms(uint32_t ms) {
    ms *= 3360; // Adjust based on your system clock
    while(ms--) {
        __NOP();
    }
}

void BNO055_Init(void) {
    // Reset BNO055
    delay_01ms(1000); // Wait for BNO055 to boot up
    
    // Check chip ID
    uint8_t chip_id = BNO055_Read_Byte(BNO055_CHIP_ID_ADDR);
    if (chip_id != 0xA0) {
        // Error handling - BNO055 not found
        while(1);
    }
    
//    // Reset system
//    BNO055_Write_Byte(BNO055_SYS_TRIGGER_ADDR, 0x20);
//    delay_01ms(6500);
    
    // Set to CONFIG mode
    BNO055_Write_Byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
    delay_01ms(2000);
    
    // Set units (degrees, m/s^2, etc)
    BNO055_Write_Byte(BNO055_UNIT_SEL_ADDR, 0x00); // Default units
    delay_01ms(1000);
    
    // Set to NDOF mode (9 Degrees of Freedom with sensor fusion)
    BNO055_Write_Byte(BNO055_OPR_MODE_ADDR,OPERATION_MODE_NDOF);
    delay_01ms(2000); // Wait for mode change
}

// Read a single byte from BNO055
uint8_t BNO055_Read_Byte(uint8_t reg_addr) {
    uint8_t data = 0;
    
    // Wait until I2C is not busy
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    
    // Send START condition
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    
    // Send device address for write
    I2C_Send7bitAddress(I2C2, BNO055_ADDRESS << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // Send register address
    I2C_SendData(I2C2, reg_addr);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // Generate restart condition
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    
    // Send device address for read
    I2C_Send7bitAddress(I2C2, BNO055_ADDRESS << 1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    // Disable acknowledge to read only one byte
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    
    // Generate STOP after data received
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    // Wait for data to be received
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    
    // Read data
    data = I2C_ReceiveData(I2C2);
    
    // Re-enable acknowledge
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    
    return data;
}

// Write a single byte to BNO055
void BNO055_Write_Byte(uint8_t reg_addr, uint8_t data) {
    // Wait until I2C is not busy
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    
    // Send START condition
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    
    // Send device address for write
    I2C_Send7bitAddress(I2C2, BNO055_ADDRESS << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // Send register address
    I2C_SendData(I2C2, reg_addr);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // Send data
    I2C_SendData(I2C2, data);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // Send STOP condition
    I2C_GenerateSTOP(I2C2, ENABLE);
}

// Read multiple bytes from BNO055
void BNO055_Read_Multiple(uint8_t reg_addr, uint8_t *buffer, uint8_t length) {
    // Wait until I2C is not busy
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
    
    // Send START condition
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    
    // Send device address for write
    I2C_Send7bitAddress(I2C2, BNO055_ADDRESS << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    
    // Send register address
    I2C_SendData(I2C2, reg_addr);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    
    // Generate restart condition
    I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    
    // Send device address for read
    I2C_Send7bitAddress(I2C2, BNO055_ADDRESS << 1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    
    // Read all but last byte with ACK
    while(length > 1) {
        // Wait for data to be received
        while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
        
        // Read data
        *buffer = I2C_ReceiveData(I2C2);
        buffer++;
        length--;
    }
    
    // Disable acknowledge for last byte
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    
    // Generate STOP after data received
    I2C_GenerateSTOP(I2C2, ENABLE);
    
    // Wait for last data to be received
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    
    // Read last byte
    *buffer = I2C_ReceiveData(I2C2);
    
    // Re-enable acknowledge
    I2C_AcknowledgeConfig(I2C2, ENABLE);
}

void BNO055_Write_Buffer(uint8_t reg_addr, uint8_t *buffer, uint8_t length) {
    // Wait until I2C bus is free
    while (I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

    // Send START condition
    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

    // Send device address for write
    I2C_Send7bitAddress(I2C2, BNO055_ADDRESS << 1, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // Send starting register address
    I2C_SendData(I2C2, reg_addr);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // Send data buffer
    for (uint8_t i = 0; i < length; i++) {
        I2C_SendData(I2C2, buffer[i]);
        while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
    }

    // Send STOP condition
    I2C_GenerateSTOP(I2C2, ENABLE);
}


// Get heading (yaw) in degrees
float BNO055_Get_Heading(void) {
 uint8_t buffer[2];
    int16_t heading;
    float heading_degrees;
    
    // Read euler heading data (yaw)
    BNO055_Read_Multiple(BNO055_EULER_H_LSB_ADDR, buffer, 2);
    
    // Convert to 16-bit signed integer (little endian)
    heading = (int16_t)((buffer[1] << 8) | buffer[0]);
    
    // Convert to degrees (BNO055 outputs in 1/16 degrees)
    heading_degrees = (float)heading / 16.0f;
	
  if (heading_degrees > 180.0f)
        return heading_degrees - 360.0f;
    else
        return heading_degrees;
    
    return heading_degrees;
}

// Get roll angle in degrees (-180 to +180)
float BNO055_Get_Roll(void) {
    uint8_t buffer[2];
    int16_t roll;
    float roll_degrees;
    
    // Read euler roll data
    BNO055_Read_Multiple(BNO055_EULER_R_LSB_ADDR, buffer, 2);
    
    // Convert to 16-bit signed integer (little endian)
    roll = (int16_t)((buffer[1] << 8) | buffer[0]);
    
    // Convert to degrees (BNO055 outputs in 1/16 degrees)
    roll_degrees = (float)roll / 16.0f;
    
    return roll_degrees;
}

// Get pitch angle in degrees (-90 to +90)
float BNO055_Get_Pitch(void) {
    uint8_t buffer[2];
    int16_t pitch;
    float pitch_degrees;
    
    // Read euler pitch data
    BNO055_Read_Multiple(BNO055_EULER_P_LSB_ADDR, buffer, 2);
    
    // Convert to 16-bit signed integer (little endian)
    pitch = (int16_t)((buffer[1] << 8) | buffer[0]);
    
    // Convert to degrees (BNO055 outputs in 1/16 degrees)
    pitch_degrees = (float)pitch / 16.0f;
    
    return pitch_degrees;
}

// Get all euler angles (heading, roll, pitch) in one function call
void BNO055_Get_Euler_Angles(float *heading, float *roll, float *pitch) {
    uint8_t buffer[6]; // Buffer to store all 6 bytes (2 bytes per angle)
    
    // Read all euler angle data at once
    BNO055_Read_Multiple(BNO055_EULER_H_LSB_ADDR, buffer, 6);
    
    // Process heading
    int16_t raw_heading = (int16_t)((buffer[1] << 8) | buffer[0]);
    *heading = (float)raw_heading / 16.0f;
    
    // Normalize heading to 0-360
//    if (*heading < 0) {
//        *heading += 360.0f;
//    }
      if (*heading > 180.0f)
        *heading += - 360.0f;
    else
		   *heading = *heading;
		
    // Process roll (-180 to +180)
    int16_t raw_roll = (int16_t)((buffer[3] << 8) | buffer[2]);
    *roll = (float)raw_roll / 16.0f;
    
    // Process pitch (-90 to +90)
    int16_t raw_pitch = (int16_t)((buffer[5] << 8) | buffer[4]);
    *pitch = (float)raw_pitch / 16.0f;
}

// Get accelerometer data in m/s²
void BNO055_Get_Accelerometer(float *x, float *y, float *z) {
    uint8_t buffer[6];
    
    // Read accelerometer data
    BNO055_Read_Multiple(BNO055_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
    
    // Convert to 16-bit signed values
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to m/s² (BNO055 outputs in 1/100 m/s²)
    *x = (float)raw_x / 100.0f;
    *y = (float)raw_y / 100.0f;
    *z = (float)raw_z / 100.0f;
}

// Get gyroscope data in degrees per second
void BNO055_Get_Gyroscope(float *x, float *y, float *z) {
    uint8_t buffer[6];
    
    // Read gyroscope data
    BNO055_Read_Multiple(BNO055_GYRO_DATA_X_LSB_ADDR, buffer, 6);
    
    // Convert to 16-bit signed values
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to degrees per second (BNO055 outputs in 1/16 degrees per second)
    *x = (float)raw_x / 16.0f;
    *y = (float)raw_y / 16.0f;
    *z = (float)raw_z / 16.0f;
}

// Get magnetometer data in micro Teslas
void BNO055_Get_Magnetometer(float *x, float *y, float *z) {
    uint8_t buffer[6];
    
    // Read magnetometer data
    BNO055_Read_Multiple(BNO055_MAG_DATA_X_LSB_ADDR, buffer, 6);
    
    // Convert to 16-bit signed values
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to micro Teslas (BNO055 outputs in 1/16 micro Teslas)
    *x = (float)raw_x / 16.0f;
    *y = (float)raw_y / 16.0f;
    *z = (float)raw_z / 16.0f;
}

// Get linear acceleration (acceleration without gravity) in m/s²
void BNO055_Get_LinearAccel(float *x, float *y, float *z) {
    uint8_t buffer[6];
    
    // Read linear acceleration data
    BNO055_Read_Multiple(BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR, buffer, 6);
    
    // Convert to 16-bit signed values
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to m/s² (BNO055 outputs in 1/100 m/s²)
    *x = (float)raw_x / 100.0f;
    *y = (float)raw_y / 100.0f;
    *z = (float)raw_z / 100.0f;
}

// Get gravity vector in m/s²
void BNO055_Get_Gravity(float *x, float *y, float *z) {
    uint8_t buffer[6];
    
    // Read gravity vector data
    BNO055_Read_Multiple(BNO055_GRAVITY_DATA_X_LSB_ADDR, buffer, 6);
    
    // Convert to 16-bit signed values
    int16_t raw_x = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_y = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_z = (int16_t)((buffer[5] << 8) | buffer[4]);
    
    // Convert to m/s² (BNO055 outputs in 1/100 m/s²)
    *x = (float)raw_x / 100.0f;
    *y = (float)raw_y / 100.0f;
    *z = (float)raw_z / 100.0f;
}

// Get all sensor data in one function call
void BNO055_Get_All_Data(BNO055_Data *data) {
    // Get accelerometer data
    BNO055_Get_Accelerometer(&data->accel.x, &data->accel.y, &data->accel.z);
    
    // Get gyroscope data
    BNO055_Get_Gyroscope(&data->gyro.x, &data->gyro.y, &data->gyro.z);
    
    // Get magnetometer data
    BNO055_Get_Magnetometer(&data->mag.x, &data->mag.y, &data->mag.z);
    
    // Get linear acceleration
    BNO055_Get_LinearAccel(&data->linearAccel.x, &data->linearAccel.y, &data->linearAccel.z);
    
    // Get gravity vector
    BNO055_Get_Gravity(&data->gravity.x, &data->gravity.y, &data->gravity.z);
    
    // Get euler angles
    BNO055_Get_Euler_Angles(&data->euler.yaw, &data->euler.roll, &data->euler.pitch);
}

void BNO055_ApplyCalibration(const BNO055_Offsets_t *offsets) {
    uint8_t calib_data[22];

    // Pack struct values into little-endian byte array
    calib_data[0]  = (uint8_t)(offsets->accel_offset_x & 0xFF);
    calib_data[1]  = (uint8_t)((offsets->accel_offset_x >> 8) & 0xFF);
    calib_data[2]  = (uint8_t)(offsets->accel_offset_y & 0xFF);
    calib_data[3]  = (uint8_t)((offsets->accel_offset_y >> 8) & 0xFF);
    calib_data[4]  = (uint8_t)(offsets->accel_offset_z & 0xFF);
    calib_data[5]  = (uint8_t)((offsets->accel_offset_z >> 8) & 0xFF);
    calib_data[6]  = (uint8_t)(offsets->mag_offset_x & 0xFF);
    calib_data[7]  = (uint8_t)((offsets->mag_offset_x >> 8) & 0xFF);
    calib_data[8]  = (uint8_t)(offsets->mag_offset_y & 0xFF);
    calib_data[9]  = (uint8_t)((offsets->mag_offset_y >> 8) & 0xFF);
    calib_data[10] = (uint8_t)(offsets->mag_offset_z & 0xFF);
    calib_data[11] = (uint8_t)((offsets->mag_offset_z >> 8) & 0xFF);
    calib_data[12] = (uint8_t)(offsets->gyro_offset_x & 0xFF);
    calib_data[13] = (uint8_t)((offsets->gyro_offset_x >> 8) & 0xFF);
    calib_data[14] = (uint8_t)(offsets->gyro_offset_y & 0xFF);
    calib_data[15] = (uint8_t)((offsets->gyro_offset_y >> 8) & 0xFF);
    calib_data[16] = (uint8_t)(offsets->gyro_offset_z & 0xFF);
    calib_data[17] = (uint8_t)((offsets->gyro_offset_z >> 8) & 0xFF);
    calib_data[18] = (uint8_t)(offsets->accel_radius & 0xFF);
    calib_data[19] = (uint8_t)((offsets->accel_radius >> 8) & 0xFF);
    calib_data[20] = (uint8_t)(offsets->mag_radius & 0xFF);
    calib_data[21] = (uint8_t)((offsets->mag_radius >> 8) & 0xFF);

		uint8_t chip_id = BNO055_Read_Byte(BNO055_CHIP_ID_ADDR);
		while (chip_id != 0xA0) {
				chip_id = BNO055_Read_Byte(BNO055_CHIP_ID_ADDR);
				delay_01ms(1000);
		}

		BNO055_Write_Byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
		delay_01ms(250);

    BNO055_Write_Buffer(REG_OFFSET_ACCEL_X_LSB, calib_data, sizeof(calib_data));
    delay_ms(100);

    BNO055_Write_Byte(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
		delay_01ms(100);

		BNO055_Write_Byte(BNO055_UNIT_SEL_ADDR, 0x00);  // default
		delay_01ms(100);

		BNO055_Write_Byte(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
		delay_01ms(200);
}

void KalmanAngle_Init(KalmanAngle *kf, float Q, float R) {
    kf->angle = 0.0f;
    kf->P = 1.0f;
    kf->Q = Q;  // nhi?u quá trình (m?c d? tin vào giá tr? hi?n t?i)
    kf->R = R;  // nhi?u do lu?ng (tin vào c?m bi?n)
}

float KalmanAngle_Update(KalmanAngle *kf, float measured_angle) {
    // D? doán
    kf->P += kf->Q;

    // Kalman gain
    kf->K = kf->P / (kf->P + kf->R);

    // C?p nh?t góc
    kf->angle += kf->K * (measured_angle - kf->angle);

    // C?p nh?t sai s?
    kf->P *= (1 - kf->K);

    return kf->angle;
}
