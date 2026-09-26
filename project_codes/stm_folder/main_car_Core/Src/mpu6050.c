/*
 * mpu6050.c
 *
 * Created on: Aug 31, 2025
 * Author: mohmm
 * Updated: Fixes calibration Z-offset truncation, retry logic bug, removes unnecessary delays
 */
#include <mpu6050.h>

// Sensitivity values for accelerometer (LSB/g)
static const float accel_sensitivity[] = {
    16384.0f, // ±2g
    8192.0f,  // ±4g
    4096.0f,  // ±8g
    2048.0f   // ±16g
};

// Sensitivity values for gyroscope (LSB/°/s)
static const float gyro_sensitivity[] = {
    131.0f, // ±250°/s
    65.5f,  // ±500°/s
    32.8f,  // ±1000°/s
    16.4f   // ±2000°/s
};

uint8_t MPU6050_Check_Connection(I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    HAL_StatusTypeDef status;

    // Check WHO_AM_I register (should return 0x68)
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 10);
    if (status != HAL_OK || check != 0x68) {
        return 1; // Connection failed
    }
    return 0; // Connection successful
}

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c, MPU6050_AccelRange accel_range, MPU6050_GyroRange gyro_range)
{
    uint8_t data;
    uint8_t retries = 3;
    uint8_t connected = 0;

    while (retries--) {
        if (MPU6050_Check_Connection(hi2c) == 0) {
            connected = 1;
            break;
        }
        HAL_Delay(5);
    }

    if (!connected) {
        return 1;
    }

    // Reset MPU6050
    data = 0x80;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 10) != HAL_OK) {
        return 1;
    }

    HAL_Delay(100);

    // Wake up MPU6050
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, 10) != HAL_OK) {
        return 1;
    }

    data = 0x07;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &data, 1, 10) != HAL_OK) {
        return 1;
    }

    data = accel_range;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &data, 1, 10) != HAL_OK) {
        return 1;
    }

    data = gyro_range;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &data, 1, 10) != HAL_OK) {
        return 1;
    }

    return 0;
}

void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data *data, uint16_t samples) {
    int32_t accel_x_sum = 0, accel_y_sum = 0, accel_z_sum = 0;
    int32_t gyro_x_sum  = 0, gyro_y_sum  = 0, gyro_z_sum  = 0;
    uint16_t i;

    // FIX: compute 1g in raw counts as an integer — avoids float-to-int16_t truncation
    // accel_sensitivity[] gives LSB/g, so 1g = sensitivity counts exactly
    uint8_t  range_idx    = data->accel_range >> 3;
    int16_t  one_g_counts = (int16_t)accel_sensitivity[range_idx];
    // e.g. ±8g  → 4096 counts/g  → one_g_counts = 4096
    //      ±4g  → 8192 counts/g  → one_g_counts = 8192
    //      ±2g  → 16384 counts/g → one_g_counts = 16384  (be careful: fits in int16_t as -32768..32767)
    // NOTE: for ±2g range, 16384 fits fine in int16_t (max positive is 32767)

    // Zero out offsets before sampling so Read_All doesn't subtract garbage
    data->accel_x_offset = 0;
    data->accel_y_offset = 0;
    data->accel_z_offset = 0;
    data->gyro_x_offset  = 0;
    data->gyro_y_offset  = 0;
    data->gyro_z_offset  = 0;

    for (i = 0; i < samples; i++) {
        MPU6050_Read_All(hi2c, data);
        accel_x_sum += data->accel_x;
        accel_y_sum += data->accel_y;
        accel_z_sum += data->accel_z - one_g_counts;  // FIX: subtract integer 1g counts, not float sensitivity
        gyro_x_sum  += data->gyro_x;
        gyro_y_sum  += data->gyro_y;
        gyro_z_sum  += data->gyro_z;
        HAL_Delay(1); // 1kHz sample rate = 1ms per sample
    }

    data->accel_x_offset = (int16_t)(accel_x_sum / samples);
    data->accel_y_offset = (int16_t)(accel_y_sum / samples);
    data->accel_z_offset = (int16_t)(accel_z_sum / samples);
    data->gyro_x_offset  = (int16_t)(gyro_x_sum  / samples);
    data->gyro_y_offset  = (int16_t)(gyro_y_sum  / samples);
    data->gyro_z_offset  = (int16_t)(gyro_z_sum  / samples);
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t recData[6];

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, recData, 6, 10) == HAL_OK) {
        data->accel_x = (int16_t)(recData[0] << 8 | recData[1]) - data->accel_x_offset;
        data->accel_y = (int16_t)(recData[2] << 8 | recData[3]) - data->accel_y_offset;
        data->accel_z = (int16_t)(recData[4] << 8 | recData[5]) - data->accel_z_offset;

        uint8_t range_idx = data->accel_range >> 3;
        data->accel_x_g = (float)data->accel_x / accel_sensitivity[range_idx];
        data->accel_y_g = (float)data->accel_y / accel_sensitivity[range_idx];
        data->accel_z_g = (float)data->accel_z / accel_sensitivity[range_idx];
    }
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t recData[6];

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, recData, 6, 10) == HAL_OK) {
        data->gyro_x = (int16_t)(recData[0] << 8 | recData[1]) - data->gyro_x_offset;
        data->gyro_y = (int16_t)(recData[2] << 8 | recData[3]) - data->gyro_y_offset;
        data->gyro_z = (int16_t)(recData[4] << 8 | recData[5]) - data->gyro_z_offset;

        uint8_t range_idx = data->gyro_range >> 3;
        data->gyro_x_dps = (float)data->gyro_x / gyro_sensitivity[range_idx];
        data->gyro_y_dps = (float)data->gyro_y / gyro_sensitivity[range_idx];
        data->gyro_z_dps = (float)data->gyro_z / gyro_sensitivity[range_idx];
    }
}

void MPU6050_Read_Temperature(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t recData[2];

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, TEMP_OUT_H_REG, 1, recData, 2, 10) == HAL_OK) {
        int16_t temp_raw = (int16_t)(recData[0] << 8 | recData[1]);
        data->temperature = (float)temp_raw / 340.0f + 36.53f;
    }
}

void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t recData[14]; // 6 (accel) + 2 (temp) + 6 (gyro)

    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, recData, 14, 10) == HAL_OK) {
        // Accelerometer
        data->accel_x = (int16_t)(recData[0] << 8 | recData[1]) - data->accel_x_offset;
        data->accel_y = (int16_t)(recData[2] << 8 | recData[3]) - data->accel_y_offset;
        data->accel_z = (int16_t)(recData[4] << 8 | recData[5]) - data->accel_z_offset;
        uint8_t range_idx = data->accel_range >> 3;
        data->accel_x_g = (float)data->accel_x / accel_sensitivity[range_idx];
        data->accel_y_g = (float)data->accel_y / accel_sensitivity[range_idx];
        data->accel_z_g = (float)data->accel_z / accel_sensitivity[range_idx];

        // Temperature
        int16_t temp_raw = (int16_t)(recData[6] << 8 | recData[7]);
        data->temperature = (float)temp_raw / 340.0f + 36.53f;

        // Gyroscope
        data->gyro_x = (int16_t)(recData[8]  << 8 | recData[9])  - data->gyro_x_offset;
        data->gyro_y = (int16_t)(recData[10] << 8 | recData[11]) - data->gyro_y_offset;
        data->gyro_z = (int16_t)(recData[12] << 8 | recData[13]) - data->gyro_z_offset;
        range_idx = data->gyro_range >> 3;
        data->gyro_x_dps = (float)data->gyro_x / gyro_sensitivity[range_idx];
        data->gyro_y_dps = (float)data->gyro_y / gyro_sensitivity[range_idx];
        data->gyro_z_dps = (float)data->gyro_z / gyro_sensitivity[range_idx];
    }
}

HAL_StatusTypeDef MPU6050_Start_Read_All_DMA(I2C_HandleTypeDef *hi2c, uint8_t *raw_data)
{
    return HAL_I2C_Mem_Read_DMA(hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, raw_data, 14);
}

void MPU6050_Process_Data(MPU6050_Data *data, uint8_t *raw_data) {
    // Accelerometer
    data->accel_x = (int16_t)(raw_data[0] << 8 | raw_data[1]) - data->accel_x_offset;
    data->accel_y = (int16_t)(raw_data[2] << 8 | raw_data[3]) - data->accel_y_offset;
    data->accel_z = (int16_t)(raw_data[4] << 8 | raw_data[5]) - data->accel_z_offset;
    uint8_t range_idx = data->accel_range >> 3;
    data->accel_x_g = (float)data->accel_x / accel_sensitivity[range_idx];
    data->accel_y_g = (float)data->accel_y / accel_sensitivity[range_idx];
    data->accel_z_g = (float)data->accel_z / accel_sensitivity[range_idx];

    // Temperature
    int16_t temp_raw = (int16_t)(raw_data[6] << 8 | raw_data[7]);
    data->temperature = (float)temp_raw / 340.0f + 36.53f;

    // Gyroscope
    data->gyro_x = (int16_t)(raw_data[8]  << 8 | raw_data[9])  - data->gyro_x_offset;
    data->gyro_y = (int16_t)(raw_data[10] << 8 | raw_data[11]) - data->gyro_y_offset;
    data->gyro_z = (int16_t)(raw_data[12] << 8 | raw_data[13]) - data->gyro_z_offset;
    range_idx = data->gyro_range >> 3;
    data->gyro_x_dps = (float)data->gyro_x / gyro_sensitivity[range_idx];
    data->gyro_y_dps = (float)data->gyro_y / gyro_sensitivity[range_idx];
    data->gyro_z_dps = (float)data->gyro_z / gyro_sensitivity[range_idx];
}
