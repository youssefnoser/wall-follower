#include "mpu6050.h"
#include <stdio.h>
#include <string.h>

/**
 * @brief Initialize MPU6050 IMU
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_Init(MPU6050_t *mpu6050) {
  uint8_t data = 0;

  // 1. Full device reset
  MPU6050_WriteRegister(mpu6050, MPU6050_PWR_MGMT_1, 0x80);
  HAL_Delay(100);

  // 2. Wake up + PLL clock from X-axis gyro
  MPU6050_WriteRegister(mpu6050, MPU6050_PWR_MGMT_1, 0x01);
  HAL_Delay(10);

  /* Verify communication by reading WHO_AM_I register (0x75) */
  MPU6050_ReadRegister(mpu6050, 0x75, &data);

  // 3. Gyro full scale ±2000 °/s (0x18)
  MPU6050_WriteRegister(mpu6050, MPU6050_GYRO_CONFIG, 0x18);

  // 4. Accel full scale ±2g (0x00)
  MPU6050_WriteRegister(mpu6050, MPU6050_ACCEL_CONFIG, 0x00);

  // 5. Sample rate = 1kHz / (1 + 4) = 200 Hz
  MPU6050_WriteRegister(mpu6050, MPU6050_SMPLRT_DIV, 0x04);
  HAL_Delay(10);

  /* Initialize data structure */
  mpu6050->accel.x = 0;
  mpu6050->accel.y = 0;
  mpu6050->accel.z = 0;
  mpu6050->gyro.x = 0;
  mpu6050->gyro.y = 0;
  mpu6050->gyro.z = 0;
  mpu6050->temperature = 0;
  mpu6050->gyro_z_offset = 0;
  mpu6050->angle_z = 0;
  mpu6050->last_time = HAL_GetTick();
}

/**
 * @brief Configure the Digital Low Pass Filter (DLPF)
 * @param mpu6050: MPU6050 handler
 * @param dlpf_cfg: DLPF config value (0-6)
 */
void MPU6050_SetDLPF(MPU6050_t *mpu6050, uint8_t dlpf_cfg) {
  MPU6050_WriteRegister(mpu6050, MPU6050_CONFIG, dlpf_cfg);
  HAL_Delay(10); /* Short delay for config to apply */
}

/**
 * @brief Write to MPU6050 register
 * @param mpu6050: MPU6050 handler
 * @param reg: Register address
 * @param value: Value to write
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_t *mpu6050, uint8_t reg,
                                        uint8_t value) {
  return HAL_I2C_Mem_Write(mpu6050->hi2c, mpu6050->i2c_addr << 1, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read from MPU6050 register
 * @param mpu6050: MPU6050 handler
 * @param reg: Register address
 * @param value: Pointer to store read value
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_t *mpu6050, uint8_t reg,
                                       uint8_t *value) {
  return HAL_I2C_Mem_Read(mpu6050->hi2c, mpu6050->i2c_addr << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Read multiple bytes from MPU6050
 * @param mpu6050: MPU6050 handler
 * @param reg: Starting register address
 * @param data: Pointer to data buffer
 * @param size: Number of bytes to read
 * @return HAL status
 */
HAL_StatusTypeDef MPU6050_ReadMultiple(MPU6050_t *mpu6050, uint8_t reg,
                                       uint8_t *data, uint16_t size) {
  memset(data, 0, size);
  return HAL_I2C_Mem_Read(mpu6050->hi2c, mpu6050->i2c_addr << 1, reg,
                          I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY);
}

/**
 * @brief Read accelerometer data
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_ReadAccelerometer(MPU6050_t *mpu6050) {
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
void MPU6050_ReadGyroscope(MPU6050_t *mpu6050) {
  uint8_t data[6];
  int16_t gyro_raw_x, gyro_raw_y, gyro_raw_z;

  MPU6050_ReadMultiple(mpu6050, MPU6050_GYRO_XOUT_H, data, 6);

  gyro_raw_x = (int16_t)((data[0] << 8) | data[1]);
  gyro_raw_y = (int16_t)((data[2] << 8) | data[3]);
  gyro_raw_z = (int16_t)((data[4] << 8) | data[5]);

  /* Convert to deg/s (using ±2000 deg/s scale) */
  mpu6050->gyro.x = gyro_raw_x / GYRO_SCALE_2000;
  mpu6050->gyro.y = gyro_raw_y / GYRO_SCALE_2000;
  mpu6050->gyro.z = gyro_raw_z / GYRO_SCALE_2000;
}

/**
 * @brief Read temperature data
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_ReadTemperature(MPU6050_t *mpu6050) {
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
void MPU6050_ReadAll(MPU6050_t *mpu6050) {
  MPU6050_ReadAccelerometer(mpu6050);
  MPU6050_ReadGyroscope(mpu6050);
  // MPU6050_ReadTemperature(mpu6050);
}

/**
 * @brief Calibrate MPU6050 Gyro Z axis
 * @param mpu6050: MPU6050 handler
 * @param iterations: Number of samples to average
 */
void MPU6050_Calibrate(MPU6050_t *mpu6050, uint16_t iterations) {
  float sum_z = 0.0f;
  for (uint16_t i = 0; i < iterations; i++) {
    MPU6050_ReadGyroscope(mpu6050);
    sum_z += mpu6050->gyro.z;
    HAL_Delay(2); // Wait 2ms between samples
  }

  if (iterations > 0) {
    mpu6050->gyro_z_offset = sum_z / (float)iterations;
  }
}

/**
 * @brief Update the absolute integrated angle based on calibrated gyro Z rate
 * @param mpu6050: MPU6050 handler
 */
void MPU6050_UpdateAngle(MPU6050_t *mpu6050) {
  uint32_t current_time = HAL_GetTick();

  // Read raw data
  MPU6050_ReadAccelerometer(mpu6050);
  MPU6050_ReadGyroscope(mpu6050);

  // Convert time delta to seconds
  float dt = (current_time - mpu6050->last_time) / 1000.0f;

  // Apply calibration offset
  float rate_z = mpu6050->gyro.z - mpu6050->gyro_z_offset;

  // // Apply a sensible deadband filter to eliminate noise
  // if (rate_z > -0.5f && rate_z < 0.5f) {
  //   rate_z = 0.0f;
  // }

  // Integrate angular velocity into absolute angle
  mpu6050->angle_z += (rate_z * dt);

  // Update timestamp for next iteration
  mpu6050->last_time = current_time;
}
