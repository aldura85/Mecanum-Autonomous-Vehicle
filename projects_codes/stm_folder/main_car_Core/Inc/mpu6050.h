#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx_hal.h" // Adjust to your STM32 HAL header (e.g., stm32f1xx_hal.h for F1 series)

// Accelerometer range enum
typedef enum {
    ACCEL_RANGE_2G  = 0x00,
    ACCEL_RANGE_4G  = 0x08,
    ACCEL_RANGE_8G  = 0x10,
    ACCEL_RANGE_16G = 0x18
} MPU6050_AccelRange;

// Gyroscope range enum
typedef enum {
    GYRO_RANGE_250  = 0x00,
    GYRO_RANGE_500  = 0x08,
    GYRO_RANGE_1000 = 0x10,
    GYRO_RANGE_2000 = 0x18
} MPU6050_GyroRange;

// MPU6050 data structure
typedef struct {
    int16_t accel_x;        // Raw X-axis accelerometer data
    int16_t accel_y;        // Raw Y-axis accelerometer data
    int16_t accel_z;        // Raw Z-axis accelerometer data
    float accel_x_g;        // X-axis acceleration in g
    float accel_y_g;        // Y-axis acceleration in g
    float accel_z_g;        // Z-axis acceleration in g
    int16_t gyro_x;         // Raw X-axis gyroscope data
    int16_t gyro_y;         // Raw Y-axis gyroscope data
    int16_t gyro_z;         // Raw Z-axis gyroscope data
    float gyro_x_dps;       // X-axis angular velocity in °/s
    float gyro_y_dps;       // Y-axis angular velocity in °/s
    float gyro_z_dps;       // Z-axis angular velocity in °/s
    float temperature;      // Temperature in °C
    MPU6050_AccelRange accel_range; // Current accelerometer range
    MPU6050_GyroRange gyro_range;   // Current gyroscope range
    int16_t accel_x_offset; // X-axis accelerometer offset
    int16_t accel_y_offset; // Y-axis accelerometer offset
    int16_t accel_z_offset; // Z-axis accelerometer offset
    int16_t gyro_x_offset;  // X-axis gyroscope offset
    int16_t gyro_y_offset;  // Y-axis gyroscope offset
    int16_t gyro_z_offset;  // Z-axis gyroscope offset
} MPU6050_Data;

// I2C address and register definitions
#define MPU6050_ADDR 0xD0
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Function prototypes
uint8_t MPU6050_Check_Connection(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_AccelRange accel_range, MPU6050_GyroRange gyro_range);
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data *data, uint16_t samples);
void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
void MPU6050_Read_Temperature(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
HAL_StatusTypeDef MPU6050_Start_Read_All_DMA(I2C_HandleTypeDef *hi2c, uint8_t *raw_data);
void MPU6050_Process_Data(MPU6050_Data *data, uint8_t *raw_data);

#endif // MPU6050_H
