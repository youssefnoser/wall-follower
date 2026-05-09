#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f1xx_hal.h"

/* MPU6050 I2C Address */
#define MPU6050_ADDR  0x68  /* Default address when AD0 = 0 */

/* MPU6050 Registers */
#define MPU6050_SMPLRT_DIV      0x19
#define MPU6050_CONFIG          0x1A  /* DLPF config register */
#define MPU6050_GYRO_CONFIG     0x1B
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_ACCEL_XOUT_L    0x3C
#define MPU6050_ACCEL_YOUT_H    0x3D
#define MPU6050_ACCEL_YOUT_L    0x3E
#define MPU6050_ACCEL_ZOUT_H    0x3F
#define MPU6050_ACCEL_ZOUT_L    0x40
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_GYRO_XOUT_L     0x44
#define MPU6050_GYRO_YOUT_H     0x45
#define MPU6050_GYRO_YOUT_L     0x46
#define MPU6050_GYRO_ZOUT_H     0x47
#define MPU6050_GYRO_ZOUT_L     0x48
#define MPU6050_TEMP_OUT_H      0x41
#define MPU6050_TEMP_OUT_L      0x42

/* Sensitivity Scales */
#define ACCEL_SCALE_2G    16384.0f
#define ACCEL_SCALE_4G    8192.0f
#define ACCEL_SCALE_8G    4096.0f
#define ACCEL_SCALE_16G   2048.0f

#define GYRO_SCALE_250    131.0f
#define GYRO_SCALE_500    65.5f
#define GYRO_SCALE_1000   32.8f
#define GYRO_SCALE_2000   16.4f

/* ---- Filter / Calibration Tuning ------------------------------------ */
#define MPU6050_COMP_ALPHA     0.98f   /* Complementary filter coefficient   */
#define MPU6050_GYRO_DEADBAND  0.3f    /* °/s — ignore gyro below this       */
#define MPU6050_DLPF_BW_44HZ  3       /* DLPF config: ~44 Hz accel/gyro BW  */
#define MPU6050_DLPF_BW_21HZ  4       /* DLPF config: ~21 Hz (smoother)     */
#define MPU6050_DLPF_BW_94HZ  2       /* DLPF config: ~94 Hz (faster)       */

typedef struct {
    float x;
    float y;
    float z;
} Vector3_t;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    Vector3_t accel;          /* Calibrated acceleration in g             */
    Vector3_t gyro;           /* Calibrated angular rate in °/s           */
    float temperature;

    /* Calibration offsets (computed at startup) */
    Vector3_t gyro_offset;    /* Zero-rate offset for gyro                */
    Vector3_t accel_offset;   /* Zero-g offset for accel (Z adjusted)    */

    /* Filtered angles (complementary filter output) */
    float pitch;              /* Rotation around X axis, degrees          */
    float roll;               /* Rotation around Y axis, degrees          */
    float yaw;                /* Rotation around Z axis, degrees (drifts) */

    /* Internal timing */
    uint32_t last_update_tick; /* HAL_GetTick() at last UpdateAngles call */
    uint8_t  calibrated;       /* 1 after calibration is done             */

    uint8_t i2c_addr;
    /* Calibration biases (in same units as accel/gyro fields) */
    Vector3_t accel_bias;
    Vector3_t gyro_bias;
    /* Filtered outputs (post-bias) */
    Vector3_t accel_filt;
    Vector3_t gyro_filt;
    /* Filter parameters */
    float filt_alpha; /* EMA alpha: 0..1 (1 -> no filtering, 0 -> hold) */
    uint8_t filt_initialized;
} MPU6050_t;

/* Function Prototypes */
void MPU6050_Init(MPU6050_t *mpu6050);
HAL_StatusTypeDef MPU6050_WriteRegister(MPU6050_t *mpu6050, uint8_t reg, uint8_t value);
HAL_StatusTypeDef MPU6050_ReadRegister(MPU6050_t *mpu6050, uint8_t reg, uint8_t *value);
HAL_StatusTypeDef MPU6050_ReadMultiple(MPU6050_t *mpu6050, uint8_t reg, uint8_t *data, uint16_t size);
void MPU6050_ReadAccelerometer(MPU6050_t *mpu6050);
void MPU6050_ReadGyroscope(MPU6050_t *mpu6050);
void MPU6050_ReadTemperature(MPU6050_t *mpu6050);
void MPU6050_ReadAll(MPU6050_t *mpu6050);

/**
 * @brief  Configure the MPU6050 built-in Digital Low-Pass Filter.
 * @param  mpu6050   MPU6050 handler
 * @param  dlpf_cfg  DLPF setting (0–6). Use MPU6050_DLPF_BW_* defines.
 *                   3 = ~44 Hz BW (good balance of noise vs latency)
 */
void MPU6050_SetDLPF(MPU6050_t *mpu6050, uint8_t dlpf_cfg);

/**
 * @brief  Calibrate gyro & accel by averaging samples while sensor is
 *         stationary and flat.  MUST be called with sensor at rest!
 * @param  mpu6050      MPU6050 handler
 * @param  num_samples  Number of samples to average (e.g. 500)
 */
void MPU6050_Calibrate(MPU6050_t *mpu6050, uint16_t num_samples);

/**
 * @brief  Read sensors, apply calibration offsets, and update pitch/roll/yaw
 *         using a complementary filter.  Call this every loop iteration.
 * @param  mpu6050  MPU6050 handler
 */
void MPU6050_UpdateAngles(MPU6050_t *mpu6050);

#endif /* MPU6050_H */
