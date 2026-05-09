#include "mpu6050.h"
#include "dwt_stm32_delay.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

MPU6050_t imu;

extern void initialise_monitor_handles(void);

void SystemClock_Config(void);
void GPIO_Init(void);
void I2C_Init(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
  initialise_monitor_handles();

  GPIO_Init();
  I2C_Init();

  printf("\n\n");
  printf("╔══════════════════════════════════════╗\n");
  printf("║       MPU6050 IMU Monitor            ║\n");
  printf("╚══════════════════════════════════════╝\n\n");

  MPU6050_Init(&imu);
  printf("[OK] MPU6050 initialized (DLPF @ 44 Hz)\n\n");

  /* ---- Calibration -------------------------------------------------- */
  MPU6050_Calibrate(&imu, 500);

  printf("[OK] Entering main loop\n\n");

  uint32_t loop_count = 0;
  uint32_t last_print_tick = HAL_GetTick();

  while (1)
  {
    /* ---- Update angles (read + filter) FREQUENTLY ------------------- */
    /* We must sample the gyro fast (e.g., every 5-10ms) for accurate
     * integration. If we only sample every 100ms, we miss rotation data
     * and the yaw will read much lower than the actual rotation! */
    MPU6050_UpdateAngles(&imu);

    /* ---- Organized print output every 100ms ------------------------- */
    uint32_t now = HAL_GetTick();
    if (now - last_print_tick >= 200)
    {
      last_print_tick = now;

      if (loop_count % 20 == 0)
      {
        printf("\n");
        printf("┌──────────────────────────────────────────────────────────────────────────────────────┐\n");
        printf("│  PITCH     ROLL      YAW     ║  Ax(g)   Ay(g)   Az(g)  ║  Gx°/s   Gy°/s   Gz°/s  │\n");
        printf("├──────────────────────────────────────────────────────────────────────────────────────┤\n");
      }

      printf("│ %+7.2f°  %+7.2f°  %+7.2f° ║ %+6.3f  %+6.3f  %+6.3f ║ %+7.2f  %+7.2f  %+7.2f │\n",
             imu.pitch, imu.roll, imu.yaw,
             imu.accel.x, imu.accel.y, imu.accel.z,
             imu.gyro.x, imu.gyro.y, imu.gyro.z);

      loop_count++;
    }
    
    /* 5ms loop = 200 Hz gyro sampling — fast enough to catch rotations */
    HAL_Delay(5);
  }

  return 0;
}

void SysTick_Handler(void) { HAL_IncTick(); }

void SystemClock_Config(void)
{
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

void GPIO_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

void I2C_Init(void)
{
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
