#ifndef BNO055_H
#define BNO055_H

#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "UGV_Timer.h"

// BNO055 I2C address
#define BNO055_ADDRESS        0x28  // Default I2C address
#define BNO055_ALT_ADDRESS    0x29  // Alternative I2C address

// BNO055 Register addresses
#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_ACCEL_DATA_X_LSB   0x08
#define BNO055_PAGE_ID_ADDR       0x07
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_CALIB_STAT_ADDR    0x35
#define BNO055_UNIT_SEL_ADDR      0x3B
#define BNO055_SYS_TRIGGER_ADDR   0x3F
#define BNO055_PWR_MODE_ADDR      0x3E
#define POWER_MODE_NORMAL         0x00
// BNO055 Operation modes
#define OPERATION_MODE_CONFIG     0x00
#define OPERATION_MODE_NDOF       0x0C  // Nine Degrees of Freedom mode with fusion
#define OPERATION_MODE_COMPASS    0x09  // Compass mode with fusion
#define OPERATION_MODE_M4G        0x0A  // Compass mode with fusion
// Accelerometer data registers
#define BNO055_ACCEL_DATA_X_LSB_ADDR      0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR      0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR      0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR      0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR      0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR      0x0D

// Gyroscope data registers
#define BNO055_GYRO_DATA_X_LSB_ADDR       0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR       0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR       0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR       0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR       0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR       0x19

// Magnetometer data registers
#define BNO055_MAG_DATA_X_LSB_ADDR        0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR        0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR        0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR        0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR        0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR        0x13

// Euler angles data registers
#define BNO055_EULER_H_LSB_ADDR   0x1A  // Heading LSB
#define BNO055_EULER_H_MSB_ADDR   0x1B  // Heading MSB
#define BNO055_EULER_R_LSB_ADDR   0x1C  // Roll LSB
#define BNO055_EULER_R_MSB_ADDR   0x1D  // Roll MSB
#define BNO055_EULER_P_LSB_ADDR   0x1E  // Pitch LSB
#define BNO055_EULER_P_MSB_ADDR   0x1F  // Pitch MSB

// Linear acceleration data registers
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR   0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR   0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR   0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR   0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR   0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR   0x2D

// Linear acceleration data registers
#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR   0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR   0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR   0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR   0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR   0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR   0x2D

// Gravity vector data registers
#define BNO055_GRAVITY_DATA_X_LSB_ADDR    0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR    0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR    0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR    0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR    0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR    0x33
// Offset registers
#define REG_OFFSET_ACCEL_X_LSB            0x55

// Private structs
typedef struct {
    float yaw; // Yaw angle in degrees (0-360)
    float roll;    // Roll angle in degrees (-180 to +180)
    float pitch;   // Pitch angle in degrees (-90 to +90)
} EulerAngles;

// Data structures for sensor readings
typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

typedef struct {
    Vector3f accel;        // Accelerometer data in m/s²
    Vector3f gyro;         // Gyroscope data in deg/s
    Vector3f mag;          // Magnetometer data in µT
    Vector3f linearAccel;  // Linear acceleration in m/s² (gravity removed)
    Vector3f gravity;      // Gravity vector in m/s²
    EulerAngles euler;     // Euler angles in degrees
    int8_t temperature;    // Temperature in degrees Celsius
} BNO055_Data;

typedef struct {
    int16_t accel_offset_x;
    int16_t accel_offset_y;
    int16_t accel_offset_z;
    int16_t mag_offset_x;
    int16_t mag_offset_y;
    int16_t mag_offset_z;
    int16_t gyro_offset_x;
    int16_t gyro_offset_y;
    int16_t gyro_offset_z;
    int16_t accel_radius;
    int16_t mag_radius;
} BNO055_Offsets_t;
// Privates variables
extern float heading;
extern EulerAngles angles;
extern BNO055_Data imu_data;
extern BNO055_Offsets_t calib_data;
// Private Function prototypes
void I2C2_Init(void);
void delay_ms(uint32_t ms);
void BNO055_Init(void);
uint8_t BNO055_Read_Byte(uint8_t reg_addr);
void BNO055_Write_Byte(uint8_t reg_addr, uint8_t data);
float BNO055_Get_Heading(void);
float BNO055_Get_Roll(void);
float BNO055_Get_Pitch(void);
void BNO055_Get_Euler_Angles(float *heading, float *roll, float *pitch);
void BNO055_Get_Accelerometer(float *x, float *y, float *z);
void BNO055_Get_Gyroscope(float *x, float *y, float *z);
void BNO055_Get_Magnetometer(float *x, float *y, float *z);
void BNO055_Get_LinearAccel(float *x, float *y, float *z);
void BNO055_Get_Gravity(float *x, float *y, float *z);
int8_t BNO055_Get_Temperature(void);
void BNO055_Get_All_Data(BNO055_Data *data);
void BNO055_ApplyCalibration(const BNO055_Offsets_t *offsets);

#endif
