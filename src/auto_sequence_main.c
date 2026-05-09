#include "dwt_stm32_delay.h"
#include "l298n.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <math.h>
#include <stdio.h>

/* Global Handlers */
L298N_t motor_driver;
MPU6050_t imu;

/* Dummy errno to fix math library linker error when semihosting is disabled */
int __errno;

/* Forward Declarations */
// extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void I2C_Init(void);

/* -----------------------------------------------------------------------
 * TUNING PARAMETERS — adjust these to match your car on your floor
 * --------------------------------------------------------------------- */
#define DRIVE_SPEED      90     /* PWM 0-100 for straight driving         */
#define TURN_SPEED       60     /* PWM 0-100 for turning (lowered to prevent overshoot) */
#define TIME_100CM    3000u     /* ms to travel ~100 cm (tune this!)      */
#define TIME_50CM     1500u     /* ms to travel ~50 cm  (tune this!)      */
#define TURN_THRESHOLD  85.0f   /* degrees: stop when |Δyaw| ≥ this value */

/* Sequence:
 *  Forward 100cm → Turn Left 90° → Forward 100cm → Stop
 */
typedef enum
{
  SEQ_CALIBRATE,
  SEQ_FWD_1,        
  SEQ_TURN_LEFT,  
  SEQ_FWD_2,        
  SEQ_STOP
} AutoState_t;

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();

  GPIO_Init();
  TIM_Init();
  I2C_Init();

  L298N_Init(&motor_driver);
  MPU6050_Init(&imu);

  AutoState_t state = SEQ_CALIBRATE;
  uint32_t state_start_time = HAL_GetTick();
  float start_yaw = 0.0f;

  while (1)
  {
    /* Always update IMU fast so turns are accurate */
    MPU6050_UpdateAngles(&imu);

    uint32_t now     = HAL_GetTick();
    uint32_t elapsed = now - state_start_time;

    switch (state)
    {
    /* ------------------------------------------------------------- */
    case SEQ_CALIBRATE:
      MPU6050_Calibrate(&imu, 500);   /* ~0.5 s, car must be still */
      HAL_Delay(1000);                /* extra settle time          */
      state = SEQ_FWD_1;
      state_start_time = HAL_GetTick();
      break;

    /* ----- Forward 1 ---------------------------------------- */
    case SEQ_FWD_1:
      L298N_Move(&motor_driver, MOTOR_LEFT,  MOTOR_FORWARD, DRIVE_SPEED);
      L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, DRIVE_SPEED);
      if (elapsed >= TIME_100CM)
      {
        L298N_StopAll(&motor_driver);
        HAL_Delay(500);
        state = SEQ_TURN_LEFT;
        start_yaw = imu.yaw;          /* snapshot yaw before turning */
      }
      break;

/* ----- Turn Left 90° ----------------------------------------- */
case SEQ_TURN_LEFT:
  L298N_Move(&motor_driver, MOTOR_LEFT,  MOTOR_BACKWARD, TURN_SPEED);
  L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD,  TURN_SPEED);
  {
    float delta = imu.yaw - start_yaw;

    /* Normalize delta to [-180, +180] to handle yaw wrap-around */
    if (delta >  180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    if (fabsf(delta) >= TURN_THRESHOLD)
    {
      L298N_StopAll(&motor_driver);
      HAL_Delay(500);
      state = SEQ_FWD_2;
      state_start_time = HAL_GetTick();
    }
  }
  break;

    /* ----- Forward 2 ----------------------------------------- */
    case SEQ_FWD_2:
      L298N_Move(&motor_driver, MOTOR_LEFT,  MOTOR_FORWARD, DRIVE_SPEED);
      L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, DRIVE_SPEED);
      if (elapsed >= TIME_100CM)
      {
        L298N_StopAll(&motor_driver);
        HAL_Delay(500);
        state = SEQ_STOP;
      }
      break;

    /* ----- Done -------------------------------------------------- */
    case SEQ_STOP:
      L298N_StopAll(&motor_driver);
      break;
    }

    HAL_Delay(5); /* 200 Hz gyro update rate */
  }

  return 0;
}

/* ------------------------------------------------------------------ */
void SysTick_Handler(void) { HAL_IncTick(); }

/* ------------------------------------------------------------------ */
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
    while (1);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    while (1);
}

/* ------------------------------------------------------------------ */
void GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  motor_driver.motor_left.IN1_PORT = GPIOB;
  motor_driver.motor_left.IN1_PIN  = GPIO_PIN_12;
  motor_driver.motor_left.IN2_PORT = GPIOB;
  motor_driver.motor_left.IN2_PIN  = GPIO_PIN_13;
  motor_driver.motor_right.IN1_PORT = GPIOB;
  motor_driver.motor_right.IN1_PIN  = GPIO_PIN_14;
  motor_driver.motor_right.IN2_PORT = GPIOB;
  motor_driver.motor_right.IN2_PIN  = GPIO_PIN_15;

  motor_driver.motor_left.PWM_Channel  = TIM_CHANNEL_1; /* PA0 */
  motor_driver.motor_right.PWM_Channel = TIM_CHANNEL_2; /* PA1 */

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = GPIO_PIN_13;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ------------------------------------------------------------------ */
void TIM_Init(void)
{
  static TIM_HandleTypeDef htim2;
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim2.Instance               = TIM2;
  htim2.Init.Prescaler         = 64 - 1;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.Period            = 1000 - 1;
  htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  motor_driver.motor_left.htim    = &htim2;
  motor_driver.motor_right.htim   = &htim2;
  motor_driver.motor_left.PWM_Max  = 1000;
  motor_driver.motor_right.PWM_Max = 1000;
}

/* ------------------------------------------------------------------ */
void I2C_Init(void)
{
  static I2C_HandleTypeDef hi2c1;

  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 400000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    while (1);

  imu.hi2c    = &hi2c1;
  imu.i2c_addr = MPU6050_ADDR;
}
