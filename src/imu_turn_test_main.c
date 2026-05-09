#include "dwt_stm32_delay.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

MPU6050_t imu;

extern void initialise_monitor_handles(void);

void SystemClock_Config(void);
void GPIO_Init(void);
void I2C_Init(void);

/* -----------------------------------------------------------------------
 * Idle-gated print buffer
 *
 * During active rotation every cycle matters — semihosting printf can
 * block for several ms per call and will cause gyro samples to be missed,
 * leading to angle undershoot.
 *
 * Strategy:
 *   - All output goes through log_printf(), which appends to a ring buffer.
 *   - In the main loop we flush one line from the buffer per iteration,
 *     BUT ONLY when the gyro has been idle for IDLE_FLUSH_MS milliseconds.
 *   - "Idle" = |gz_raw| below IDLE_THRESHOLD_RAW for IDLE_CONFIRM_SAMPLES
 *     consecutive samples.
 * ----------------------------------------------------------------------- */

#define LOG_BUF_SIZE 4096
#define IDLE_THRESHOLD_RAW                                                     \
  300 /* raw LSB — well above noise (~90), well below a real turn */
#define IDLE_CONFIRM_MS                                                        \
  150 /* gyro must be calm for this long before we flush */

static char log_buf[LOG_BUF_SIZE];
static int log_head = 0; /* next write position */
static int log_tail = 0; /* next read  position */
static int log_full = 0;

static void log_printf(const char *fmt, ...) {
  char tmp[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end(args);

  for (int i = 0; i < len && !log_full; i++) {
    log_buf[log_head] = tmp[i];
    log_head = (log_head + 1) % LOG_BUF_SIZE;
    if (log_head == log_tail)
      log_full = 1;
  }
}

/* Flush one complete line (\n-terminated) from the buffer.
 * Returns 1 if something was printed, 0 if buffer empty. */
static int log_flush_one_line(void) {
  if (log_head == log_tail && !log_full)
    return 0;

  char line[256];
  int i = 0;
  while ((log_head != log_tail || log_full) && i < (int)sizeof(line) - 1) {
    char c = log_buf[log_tail];
    log_tail = (log_tail + 1) % LOG_BUF_SIZE;
    log_full = 0;
    line[i++] = c;
    if (c == '\n')
      break;
  }
  line[i] = '\0';
  printf("%s", line); /* single semihosting call per line */
  return 1;
}

/* -----------------------------------------------------------------------
 * Idle detection — call every loop iteration with latest raw gz
 * Returns 1 when the gyro has been continuously idle for IDLE_CONFIRM_MS
 * ----------------------------------------------------------------------- */
static uint8_t gyro_is_idle(float gz) {
  static uint32_t idle_since = 0;
  static uint8_t was_idle = 0;

  if (gz > 2.5f || gz < -2.5f) {
    idle_since = HAL_GetTick();
    was_idle = 0;
    return 0;
  }

  if (!was_idle) {
    if ((HAL_GetTick() - idle_since) >= IDLE_CONFIRM_MS)
      was_idle = 1;
  }
  return was_idle;
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
  initialise_monitor_handles();

  GPIO_Init();
  I2C_Init();

  /* These go out immediately — we're not in the hot loop yet */
  printf("\n\n");
  printf("╔══════════════════════════════════════╗\n");
  printf("║       MPU6050 IMU Monitor            ║\n");
  printf("╚══════════════════════════════════════╝\n\n");

  MPU6050_Init(&imu);
    MPU6050_SetDLPF(&imu, MPU6050_DLPF_44HZ);

  MPU6050_Calibrate(&imu, 500);

  printf("[OK] Entering main loop — output suppressed during motion\n\n");

  uint32_t last_log_tick = HAL_GetTick();
  uint32_t loop_count = 0;

  while (1) {
    /* ---- 1. Sample gyro as fast as possible ---- */
    MPU6050_UpdateAngle(&imu);

    uint8_t idle = gyro_is_idle(imu.gyro.z);

    /* ---- 3. Queue a log line every 200 ms ---- */
    uint32_t now = HAL_GetTick();
    if (now - last_log_tick >= 200) {
      last_log_tick = now;

      if (loop_count % 20 == 0) {
        log_printf("\n");
        log_printf("┌────────────────────────────────────────────────────────"
                   "──────────────┐\n");
        log_printf("│ ANGLE_Z ║  Ax(g)   Ay(g)   Az(g)  ║ "
                   " Gx°/s   Gy°/s   Gz°/s  │\n");
        log_printf("├────────────────────────────────────────────────────────"
                   "──────────────┤\n");
      }

      log_printf("│ %+7.2f° ║ %+6.3f  %+6.3f  %+6.3f ║ "
                 "%+7.2f  %+7.2f  %+7.2f │\n",
                 imu.angle_z, imu.accel.x, imu.accel.y, imu.accel.z, imu.gyro.x,
                 imu.gyro.y, imu.gyro.z);

      loop_count++;
    }

    /* ---- 4. Flush one line only when idle ---- */
    if (idle) {
      log_flush_one_line();
    }

    /* ---- 5. Fixed 5 ms loop = 200 Hz gyro sampling ---- */
    HAL_Delay(5);
  }

  return 0;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void I2C_Init(void) {
  static I2C_HandleTypeDef hi2c1;

  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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