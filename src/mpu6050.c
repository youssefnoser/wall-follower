#include "mpu6050.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG  (180.0f / M_PI)

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
    
    /* Enable built-in Digital Low-Pass Filter (DLPF)
     * Setting 3 = ~44 Hz bandwidth — good balance of noise removal vs latency.
     * This filters both accel and gyro at the hardware level before we read. */
    MPU6050_SetDLPF(mpu6050, MPU6050_DLPF_BW_44HZ);
    
    /* Set sample rate = 1 kHz / (1 + 0) = 1 kHz
     * (DLPF enabled → gyro output = 1 kHz, not 8 kHz) */
    MPU6050_WriteRegister(mpu6050, MPU6050_SMPLRT_DIV, 0x00);
    
    /* Initialize data structure */
    mpu6050->accel.x = 0;
    mpu6050->accel.y = 0;
    mpu6050->accel.z = 0;
    mpu6050->gyro.x = 0;
    mpu6050->gyro.y = 0;
    mpu6050->gyro.z = 0;
    mpu6050->temperature = 0;

    /* Zero calibration offsets */
    mpu6050->gyro_offset.x  = 0;
    mpu6050->gyro_offset.y  = 0;
    mpu6050->gyro_offset.z  = 0;
    mpu6050->accel_offset.x = 0;
    mpu6050->accel_offset.y = 0;
    mpu6050->accel_offset.z = 0;

    /* Zero filtered angles */
    mpu6050->pitch = 0;
    mpu6050->roll  = 0;
    mpu6050->yaw   = 0;

    mpu6050->last_update_tick = HAL_GetTick();
    mpu6050->calibrated = 0;
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
 * @brief Configure the MPU6050 built-in Digital Low-Pass Filter
 * @param mpu6050: MPU6050 handler
 * @param dlpf_cfg: DLPF setting (0–6)
 *   0 = 260 Hz (essentially off)
 *   1 = 184 Hz
 *   2 =  94 Hz
 *   3 =  44 Hz  ← recommended
 *   4 =  21 Hz
 *   5 =  10 Hz
 *   6 =   5 Hz
 */
void MPU6050_SetDLPF(MPU6050_t *mpu6050, uint8_t dlpf_cfg)
{
    if (dlpf_cfg > 6) dlpf_cfg = 6;
    MPU6050_WriteRegister(mpu6050, MPU6050_CONFIG, dlpf_cfg);
}

/**
 * @brief Read accelerometer data (raw, before calibration offset)
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
    
    /* Convert to g (using ±2g scale) then subtract calibration offset */
    mpu6050->accel.x = (accel_raw_x / ACCEL_SCALE_2G) - mpu6050->accel_offset.x;
    mpu6050->accel.y = (accel_raw_y / ACCEL_SCALE_2G) - mpu6050->accel_offset.y;
    mpu6050->accel.z = (accel_raw_z / ACCEL_SCALE_2G) - mpu6050->accel_offset.z;
}

/**
 * @brief Read gyroscope data (raw, before calibration offset)
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
    
    /* Convert to deg/s (using ±250 deg/s scale) then subtract calibration offset */
    mpu6050->gyro.x = (gyro_raw_x / GYRO_SCALE_250) - mpu6050->gyro_offset.x;
    mpu6050->gyro.y = (gyro_raw_y / GYRO_SCALE_250) - mpu6050->gyro_offset.y;
    mpu6050->gyro.z = (gyro_raw_z / GYRO_SCALE_250) - mpu6050->gyro_offset.z;
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

/**
 * @brief  Calibrate gyro & accel by averaging samples while sensor is at rest.
 *
 *         The sensor MUST be stationary and flat (Z pointing up) during this!
 *         Gyro offset: average of all readings (should be ~0 when still).
 *         Accel offset: average of X,Y (should be ~0) and Z adjusted for 1g gravity.
 *
 * @param  mpu6050      MPU6050 handler
 * @param  num_samples  Number of samples to average (e.g. 500)
 */
void MPU6050_Calibrate(MPU6050_t *mpu6050, uint16_t num_samples)
{
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;

    printf("\n========================================\n");
    printf("  CALIBRATING MPU6050 — DO NOT MOVE!\n");
    printf("  Averaging %d samples...\n", num_samples);
    printf("========================================\n");

    /* Temporarily clear offsets so we read raw values */
    mpu6050->gyro_offset.x  = 0;
    mpu6050->gyro_offset.y  = 0;
    mpu6050->gyro_offset.z  = 0;
    mpu6050->accel_offset.x = 0;
    mpu6050->accel_offset.y = 0;
    mpu6050->accel_offset.z = 0;

    /* Discard first few readings (sensor settling) */
    for (uint16_t i = 0; i < 50; i++) {
        MPU6050_ReadAll(mpu6050);
        HAL_Delay(2);
    }

    /* Accumulate samples */
    for (uint16_t i = 0; i < num_samples; i++) {
        MPU6050_ReadAll(mpu6050);

        sum_gx += mpu6050->gyro.x;
        sum_gy += mpu6050->gyro.y;
        sum_gz += mpu6050->gyro.z;

        sum_ax += mpu6050->accel.x;
        sum_ay += mpu6050->accel.y;
        sum_az += mpu6050->accel.z;

        HAL_Delay(2);  /* ~2 ms between samples → ~1 second for 500 samples */
    }

    /* Compute averages → these become the offsets */
    mpu6050->gyro_offset.x = sum_gx / (float)num_samples;
    mpu6050->gyro_offset.y = sum_gy / (float)num_samples;
    mpu6050->gyro_offset.z = sum_gz / (float)num_samples;

    mpu6050->accel_offset.x = sum_ax / (float)num_samples;
    mpu6050->accel_offset.y = sum_ay / (float)num_samples;
    /* Z-axis: gravity reads ~+1.0g when flat, so offset = average - 1.0 */
    mpu6050->accel_offset.z = (sum_az / (float)num_samples) - 1.0f;

    mpu6050->calibrated = 1;

    /* Seed the angles from the accelerometer so we don't start at 0 */
    MPU6050_ReadAll(mpu6050);
    float ax = mpu6050->accel.x;
    float ay = mpu6050->accel.y;
    float az = mpu6050->accel.z;
    mpu6050->pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    mpu6050->roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;
    mpu6050->yaw   = 0.0f;

    mpu6050->last_update_tick = HAL_GetTick();

    printf("\n  Gyro  offsets: X=%+.3f  Y=%+.3f  Z=%+.3f deg/s\n",
           mpu6050->gyro_offset.x, mpu6050->gyro_offset.y, mpu6050->gyro_offset.z);
    printf("  Accel offsets: X=%+.4f  Y=%+.4f  Z=%+.4f g\n",
           mpu6050->accel_offset.x, mpu6050->accel_offset.y, mpu6050->accel_offset.z);
    printf("  Initial angles: P=%+.1f  R=%+.1f  Y=%+.1f deg\n",
           mpu6050->pitch, mpu6050->roll, mpu6050->yaw);
    printf("========================================\n");
    printf("  CALIBRATION COMPLETE!\n");
    printf("========================================\n\n");
}

/**
 * @brief  Read sensors, apply calibration, and compute pitch/roll/yaw
 *         using a complementary filter.
 *
 *         Pitch & Roll: fuses gyro integration (fast, drifts) with
 *         accelerometer angle (noisy, no drift) → stable in both
 *         short-term and long-term.
 *
 *         Yaw: gyro integration only (with deadband).  Will drift
 *         over time — this is a hardware limitation without a magnetometer.
 *
 * @param  mpu6050  MPU6050 handler
 */
void MPU6050_UpdateAngles(MPU6050_t *mpu6050)
{
    /* ---- Read calibrated sensor data -------------------------------- */
    MPU6050_ReadAll(mpu6050);

    /* ---- Compute dt (seconds) --------------------------------------- */
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - mpu6050->last_update_tick) * 0.001f;
    mpu6050->last_update_tick = now;

    /* Guard against wildly wrong dt (first call, overflow, etc.) */
    if (dt <= 0.0f || dt > 1.0f)
        dt = 0.01f;

    float ax = mpu6050->accel.x;
    float ay = mpu6050->accel.y;
    float az = mpu6050->accel.z;
    float gx = mpu6050->gyro.x;
    float gy = mpu6050->gyro.y;
    float gz = mpu6050->gyro.z;

    /* ---- Accelerometer-based angles (reference from gravity) -------- */
    float accel_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
    float accel_roll  = atan2f(ay, sqrtf(ax * ax + az * az)) * RAD_TO_DEG;

    /* ---- Complementary filter for Pitch & Roll ---------------------- */
    /*   High-pass (gyro): tracks fast changes, but drifts long-term.
     *   Low-pass  (accel): provides absolute reference, but is noisy.
     *   α = 0.98 → trust gyro 98%, accel 2% each cycle.                */
    mpu6050->pitch = MPU6050_COMP_ALPHA * (mpu6050->pitch + gx * dt)
                   + (1.0f - MPU6050_COMP_ALPHA) * accel_pitch;

    mpu6050->roll  = MPU6050_COMP_ALPHA * (mpu6050->roll  + gy * dt)
                   + (1.0f - MPU6050_COMP_ALPHA) * accel_roll;

    /* ---- Yaw: gyro integration only (with deadband) ----------------- */
    /*   No magnetometer → no absolute yaw reference → drift is expected.
     *   Deadband suppresses integration of sensor noise when stationary. */
    if (gz > MPU6050_GYRO_DEADBAND || gz < -MPU6050_GYRO_DEADBAND) {
        mpu6050->yaw += gz * dt;
    }
}
