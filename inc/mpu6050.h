#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

/* MPU6050 I2C Address */
#define MPU6050_ADDR 0x68 /* Default address when AD0 = 0 */

/* MPU6050 Registers */
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_TEMP_OUT_H 0x41
#define MPU6050_TEMP_OUT_L 0x42

/* Sensitivity Scales */
#define ACCEL_SCALE_2G 16384.0f
#define ACCEL_SCALE_4G 8192.0f
#define ACCEL_SCALE_8G 4096.0f
#define ACCEL_SCALE_16G 2048.0f

#define GYRO_SCALE_250 131.0f
#define GYRO_SCALE_500 65.5f
#define GYRO_SCALE_1000 32.8f
#define GYRO_SCALE_2000 16.4f

typedef struct {
  float x;
  float y;
  float z;
} Vector3_t;

typedef struct {
  I2C_HandleTypeDef *hi2c;
  Vector3_t accel;
  Vector3_t gyro;
  float temperature;
  uint8_t i2c_addr;
  /* Added for calibration and integration */
  float gyro_z_offset;
  float angle_z;
  uint32_t last_time;
} MPU6050_t;

/* Function Prototypes */
void MPU6050_Init(MPU6050_t *mpu6050);
void MPU6050_Calibrate(MPU6050_t *mpu6050, uint16_t iterations);
void MPU6050_UpdateAngle(MPU6050_t *mpu6050);
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_t *mpu6050, uint8_t reg,
                                        uint8_t value);
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_t *mpu6050, uint8_t reg,
                                       uint8_t *value);
HAL_StatusTypeDef MPU6050_ReadMultiple(MPU6050_t *mpu6050, uint8_t reg,
                                       uint8_t *data, uint16_t size);
void MPU6050_ReadAccelerometer(MPU6050_t *mpu6050);
void MPU6050_ReadGyroscope(MPU6050_t *mpu6050);
void MPU6050_ReadTemperature(MPU6050_t *mpu6050);
void MPU6050_ReadAll(MPU6050_t *mpu6050);

#endif /* MPU6050_H */
