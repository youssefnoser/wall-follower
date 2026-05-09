#include "dwt_stm32_delay.h"
#include "esp_ring.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

UART_HandleTypeDef huart1;
MPU6050_t imu;

extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
void I2C_Init(void);
void USART1_UART_Init(void);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
  initialise_monitor_handles(); // Initialize semihosting if enabled
  USART1_UART_Init();

  I2C_Init();

  printf("IMU initialization sequence starting...\n");
  MPU6050_Init(&imu);

  uint8_t dlpf_config = MPU6050_DLPF_5HZ; /* Configurable DLPF value */
  MPU6050_SetDLPF(&imu, dlpf_config);

  printf("Calibrating IMU... DO NOT MOVE THE DEVICE.\n");
  MPU6050_Calibrate(&imu, 1000); // Collect 1000 samples for around 2 seconds
  printf("Calibration complete! Z-Axis offset: %.2f\n", imu.gyro_z_offset);

  printf("Starting integration loop...\n");
  imu.last_time = HAL_GetTick(); // Sync timing base before loop

  while (1) {
    MPU6050_UpdateAngle(&imu);

    printf("Rate Z: %7.1f deg/s | Angle Z: %7.1f deg\r\n",
           (imu.gyro.z - imu.gyro_z_offset), imu.angle_z);

    HAL_Delay(20); // 50 Hz update rate
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

void USART1_UART_Init(void) {
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    while (1)
      ;

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
