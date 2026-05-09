/*
 * motor_imu_test_main.c
 *
 * Self-contained bench test — no WiFi.
 *
 * Sequence
 * ────────
 *  1. IMU calibrate (keep still).
 *  2. RAMP UP   both motors: 10 → 20 → 30 → … → 90 %, 600 ms per step.
 *  3. RAMP DOWN both motors: 80 → 70 → … → 10 %, 600 ms per step.
 *  4. STOP  (1 s pause so the car settles).
 *  5. TURN  closed-loop +90 °  (CCW when viewed from above).
 *  6. Sit in idle loop printing angle every 500 ms.
 */

#include "dwt_stm32_delay.h"
#include "flash_logger.h"
#include "l298n.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
/* ── tunables ─────────────────────────────────────────────────────── */
#define RAMP_STEP_MS 600 /* dwell time at each speed step (ms)    */
#define RAMP_STEP_PCT 10 /* increment: 10, 20, … 90               */
#define RAMP_MAX_PCT 90

#define TURN_TARGET_DEG 90.0f   /* how far to rotate                     */
#define TURN_SPEED 35           /* % motor power during turn             */
#define TURN_SLOWDOWN_DEG 15.0f /* ramp to half speed in last N degrees  */
#define TURN_TIMEOUT_MS 6000    /* safety cut-off                        */

/* ── globals ──────────────────────────────────────────────────────── */
L298N_t motor_driver;
MPU6050_t imu;

extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void I2C_Init(void);

/* ================================================================== */
/*  Motor helpers                                                      */
/* ================================================================== */
static void apply_motor(uint8_t motor, int pct) {
  if (pct > 100)
    pct = 100;
  if (pct < -100)
    pct = -100;

  if (pct == 0) {
    L298N_Stop(&motor_driver, motor);
    return;
  }

  uint8_t dir = (pct > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint8_t speed = (uint8_t)(pct > 0 ? pct : -pct);
  L298N_Move(&motor_driver, motor, dir, speed);
}

static void both_motors(int pct) {
  apply_motor(MOTOR_LEFT, pct);
  apply_motor(MOTOR_RIGHT, pct);
}

/* ================================================================== */
/*  Ramp sequence                                                      */
/* ================================================================== */
static void ramp_up(void) {
  printf("[RAMP] UP  ───────────────────────\n");
  for (int pct = RAMP_STEP_PCT; pct <= RAMP_MAX_PCT; pct += RAMP_STEP_PCT) {
    printf("[RAMP] %d %%\n", pct);
    both_motors(pct);
    HAL_Delay(RAMP_STEP_MS);
  }
}

static void ramp_down(void) {
  printf("[RAMP] DOWN ──────────────────────\n");
  for (int pct = RAMP_MAX_PCT - RAMP_STEP_PCT; pct >= RAMP_STEP_PCT;
       pct -= RAMP_STEP_PCT) {
    printf("[RAMP] %d %%\n", pct);
    both_motors(pct);
    HAL_Delay(RAMP_STEP_MS);
  }
  L298N_StopAll(&motor_driver);
  printf("[RAMP] STOP\n");
}

/* ================================================================== */
/*  Closed-loop turn                                                   */
/*                                                                     */
/*  Tank-spins until angle_z changes by `delta` degrees.              */
/*  Positive delta = CCW (left spin).                                 */
/* ================================================================== */
static void do_turn(float delta) {
  if (fabsf(delta) < 1.0f) {
    printf("[TURN] delta too small, skipped\n");
    return;
  }

  /* fresh angle snapshot */
  MPU6050_UpdateAngle(&imu);

  if (fabsf(imu.angle_z) > 5.0f) {
    printf("[TURN] ERR: start angle %.2f too large (not zeroed?). Aborting.\n",
           imu.angle_z);
    return;
  }

  float target = imu.angle_z + delta;

  printf("[TURN] start=%.2f  target=%.2f  delta=%.2f deg\n", imu.angle_z,
         target, delta);

  uint32_t t_start = HAL_GetTick();
  uint32_t last_print = t_start;

  while (1) {
    MPU6050_UpdateAngle(&imu);
    float current = imu.angle_z;
    float remaining = target - current; /* signed */

    /* done? (with 2.0 deg deadband to handle oscillation) */
    bool done =
        (delta > 0) ? (current >= target - 2.0f) : (current <= target + 2.0f);
    if (done)
      break;

    /* safety timeout */
    if ((HAL_GetTick() - t_start) >= TURN_TIMEOUT_MS) {
      printf("[TURN] TIMEOUT — stopping\n");
      break;
    }

    /* adaptive speed: linear ramp-down in last TURN_SLOWDOWN_DEG */
    float abs_rem = fabsf(remaining);
    int spd = TURN_SPEED;
    if (abs_rem < TURN_SLOWDOWN_DEG) {
      spd = (int)(TURN_SPEED * (0.5f + 0.5f * (abs_rem / TURN_SLOWDOWN_DEG)));
      if (spd < 20)
        spd = 20; /* floor: must beat static friction */
    }

    /* CCW (positive delta): left backward, right forward */
    if (delta > 0) {
      apply_motor(MOTOR_LEFT, -spd);
      apply_motor(MOTOR_RIGHT, spd);
    } else {
      apply_motor(MOTOR_LEFT, spd);
      apply_motor(MOTOR_RIGHT, -spd);
    }

    /* live print every 100 ms */
    if (HAL_GetTick() - last_print >= 100) {
      last_print = HAL_GetTick();
      printf("[TURN]  angle=%+7.2f°  remaining=%+6.2f°  spd=%d%%\n", current,
             remaining, spd);
    }
  }

  L298N_StopAll(&motor_driver);

  float error = imu.angle_z - target;
  printf("[TURN] done  final=%+.2f°  target=%+.2f°  error=%+.2f°\n",
         imu.angle_z, target, error);
}

/* ================================================================== */
/*  main                                                               */
/* ================================================================== */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
  // initialise_monitor_handles();
  flash_logger_init();

  GPIO_Init();
  TIM_Init();
  I2C_Init();

  L298N_Init(&motor_driver);

  /* ── IMU ──────────────────────────────────────────────────────── */
  MPU6050_Init(&imu);
  MPU6050_SetDLPF(&imu, MPU6050_DLPF_44HZ);
  printf("[IMU] Calibrating — keep still...\n");
  MPU6050_Calibrate(&imu, 500);
  printf("[IMU] Done. gyro_z_offset=%.4f\n", imu.gyro_z_offset);

  // /* ── 1. Ramp up ───────────────────────────────────────────────── */
  // ramp_up();

  // /* ── 2. Ramp down ─────────────────────────────────────────────── */
  // ramp_down();

  /* ── 3. Settle pause ──────────────────────────────────────────── */
  printf("[TEST] settling 1 s before turn...\n");
  HAL_Delay(1000);

  /* ── 4. Reset heading reference, then 90 ° CCW turn ──────────── */
  imu.angle_z = 0.0f;
  imu.last_time = HAL_GetTick();
  printf("[TEST] angle_z zeroed. Executing 90 deg CCW turn.\n");
  do_turn(TURN_TARGET_DEG);

  /* ── 5. Idle — print angle every 500 ms ──────────────────────── */
  printf("[TEST] Complete. Idling.\n");
  uint32_t last = HAL_GetTick();
  while (1) {
    MPU6050_UpdateAngle(&imu);
    if (HAL_GetTick() - last >= 500) {
      last = HAL_GetTick();
      printf("[IDLE] angle_z=%+.2f°\n", imu.angle_z);
    }
  }
}

/* ================================================================== */
/*  Peripheral init                                                    */
/* ================================================================== */
void I2C_Init(void) {
  static I2C_HandleTypeDef hi2c1;

  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_OD;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    while (1)
      ;

  imu.hi2c = &hi2c1;
  imu.i2c_addr = MPU6050_ADDR;
}

void SysTick_Handler(void) { HAL_IncTick(); }

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    while (1)
      ;

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    while (1)
      ;
}

void GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  motor_driver.motor_left.IN1_PORT = GPIOB;
  motor_driver.motor_left.IN1_PIN = GPIO_PIN_12;
  motor_driver.motor_left.IN2_PORT = GPIOB;
  motor_driver.motor_left.IN2_PIN = GPIO_PIN_13;
  motor_driver.motor_right.IN1_PORT = GPIOB;
  motor_driver.motor_right.IN1_PIN = GPIO_PIN_14;
  motor_driver.motor_right.IN2_PORT = GPIOB;
  motor_driver.motor_right.IN2_PIN = GPIO_PIN_15;

  motor_driver.motor_left.PWM_Channel = TIM_CHANNEL_1;
  motor_driver.motor_right.PWM_Channel = TIM_CHANNEL_2;

  /* onboard LED */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
}

void TIM_Init(void) {
  static TIM_HandleTypeDef htim2;
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64 - 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  motor_driver.motor_left.htim = &htim2;
  motor_driver.motor_right.htim = &htim2;
  motor_driver.motor_left.PWM_Max = 1000;
  motor_driver.motor_right.PWM_Max = 1000;
}