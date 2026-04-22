#include "mpu6050.h"

/**
 * @brief Initialize MPU6050 IMU
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_Init(MPU6050_t *mpu6050)
{
    uint8_t data = 0;
    
    /* Wake up the MPU6050 (clear sleep bit) */
    MPU6050_WriteRegister(mpu6050, MPU6050_PWR_MGMT_1, 0x00);
    
    HAL_Delay(100);
    
    /* Verify communication by reading WHO_AM_I register (0x75) */
    MPU6050_ReadRegister(mpu6050, 0x75, &data);
    
    /* Initialize data structure */
    mpu6050->accel.x = 0;
    mpu6050->accel.y = 0;
    mpu6050->accel.z = 0;
    mpu6050->gyro.x = 0;
    mpu6050->gyro.y = 0;
    mpu6050->gyro.z = 0;
    mpu6050->temperature = 0;
}

/**
 * @brief Write to MPU6050 register
 * @param mpu6050: MPU6050 handler
 * @param reg: Register address
 * @param value: Value to write
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_t *mpu6050, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(mpu6050->hi2c, 
                             mpu6050->i2c_addr << 1, 
                             reg, 
                             I2C_MEMADD_SIZE_8BIT, 
                             &value, 
                             1, 
                             HAL_MAX_DELAY);
}

/**
 * @brief Read from MPU6050 register
 * @param mpu6050: MPU6050 handler
 * @param reg: Register address
 * @param value: Pointer to store read value
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_t *mpu6050, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(mpu6050->hi2c, 
                            mpu6050->i2c_addr << 1, 
                            reg, 
                            I2C_MEMADD_SIZE_8BIT, 
                            value, 
                            1, 
                            HAL_MAX_DELAY);
}

/**
 * @brief Read multiple bytes from MPU6050
 * @param mpu6050: MPU6050 handler
 * @param reg: Starting register address
 * @param data: Pointer to data buffer
 * @param size: Number of bytes to read
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadMultiple(MPU6050_t *mpu6050, uint8_t reg, uint8_t *data, uint16_t size)
{
    return HAL_I2C_Mem_Read(mpu6050->hi2c, 
                            mpu6050->i2c_addr << 1, 
                            reg, 
                            I2C_MEMADD_SIZE_8BIT, 
                            data, 
                            size, 
                            HAL_MAX_DELAY);
}

/**
 * @brief Read accelerometer data
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_ReadAccelerometer(MPU6050_t *mpu6050)
{
    uint8_t data[6];
    int16_t accel_raw_x, accel_raw_y, accel_raw_z;
    
    MPU6050_ReadMultiple(mpu6050, MPU6050_ACCEL_XOUT_H, data, 6);
    
    accel_raw_x = (int16_t)((data[0] << 8) | data[1]);
    accel_raw_y = (int16_t)((data[2] << 8) | data[3]);
    accel_raw_z = (int16_t)((data[4] << 8) | data[5]);
    
    /* Convert to g (using ±2g scale) */
    mpu6050->accel.x = accel_raw_x / ACCEL_SCALE_2G;
    mpu6050->accel.y = accel_raw_y / ACCEL_SCALE_2G;
    mpu6050->accel.z = accel_raw_z / ACCEL_SCALE_2G;
}

/**
 * @brief Read gyroscope data
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_ReadGyroscope(MPU6050_t *mpu6050)
{
    uint8_t data[6];
    int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;
    
    MPU6050_ReadMultiple(mpu6050, MPU6050_GYRO_XOUT_H, data, 6);
    
    gyro_raw_x = (int16_t)((data[0] << 8) | data[1]);
    gyro_raw_y = (int16_t)((data[2] << 8) | data[3]);
    gyro_raw_z = (int16_t)((data[4] << 8) | data[5]);
    
    /* Convert to deg/s (using ±250 deg/s scale) */
    mpu6050->gyro.x = gyro_raw_x / GYRO_SCALE_250;
    mpu6050->gyro.y = gyro_raw_y / GYRO_SCALE_250;
    mpu6050->gyro.z = gyro_raw_z / GYRO_SCALE_250;
}

/**
 * @brief Read temperature data
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_ReadTemperature(MPU6050_t *mpu6050)
{
    uint8_t data[2];
    int16_t temp_raw;
    
    MPU6050_ReadMultiple(mpu6050, MPU6050_TEMP_OUT_H, data, 2);
    
    temp_raw = (int16_t)((data[0] << 8) | data[1]);
    
    /* Temperature = (TEMP_OUT / 340) + 36.53 */
    mpu6050->temperature = (temp_raw / 340.0f) + 36.53f;
}

/**
 * @brief Read all sensor data (accel, gyro, temp)
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_ReadAll(MPU6050_t *mpu6050)
{
    MPU6050_ReadAccelerometer(mpu6050);
    MPU6050_ReadGyroscope(mpu6050);
    // MPU6050_ReadTemperature(mpu6050);
}
