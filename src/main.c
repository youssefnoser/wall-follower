#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/* Global Handlers */
L298N_t motor_driver;

/* Three HC-SR04 sensors on TIM3 CH1/CH2/CH3
 *
 *  Sensor        TRIG    ECHO         TIM3 Channel
 *  ──────────────────────────────────────────────
 *  Front         PB8     PA6 (AF)     CH1
 *  Left          PB9     PA7 (AF)     CH2
 *  Right         PB10    PB0 (AF)     CH3
 *
 * NOTE: PB8/PB9/PB10 are plain GPIO outputs — any free pins work here.
 *       PA6/PA7/PB0 MUST be the alternate-function pins for TIM3 CH1-3
 *       on the STM32F103 (no remapping needed for these three).
 */
HC_SR04_t ultrasonic_front;
HC_SR04_t ultrasonic_left;
HC_SR04_t ultrasonic_right;

IR_Sensor_t ir_sensor_left;
IR_Sensor_t ir_sensor_right;
MPU6050_t imu;

/* Forward Declarations */
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void I2C_Init(void);
void ADC_Init(void);

/* ------------------------------------------------------------------ */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  initialise_monitor_handles();

  GPIO_Init();
  TIM_Init();
  I2C_Init();
  ADC_Init();

  HC_SR04_Init(&ultrasonic_front);
  HC_SR04_Init(&ultrasonic_left);
  HC_SR04_Init(&ultrasonic_right);

  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    /* Fire each sensor with a small gap so echoes don't overlap.
     * 50 ms is comfortably longer than the max echo time (~38 ms at 6.5 m). */
    HC_SR04_StartMeasurement(&ultrasonic_front);
    HAL_Delay(50);

    HC_SR04_StartMeasurement(&ultrasonic_left);
    HAL_Delay(50);

    HC_SR04_StartMeasurement(&ultrasonic_right);
    HAL_Delay(50);

    volatile float dist_front = HC_SR04_GetDistance(&ultrasonic_front);
    volatile float dist_left = HC_SR04_GetDistance(&ultrasonic_left);
    volatile float dist_right = HC_SR04_GetDistance(&ultrasonic_right);

    printf("Front: %.2f cm  |  Left: %.2f cm  |  Right: %.2f cm\n", dist_front,
           dist_left, dist_right);

    HAL_Delay(100);
  }

  return 0;
}

/* ------------------------------------------------------------------ */
void SysTick_Handler(void) { HAL_IncTick(); }

/* ------------------------------------------------------------------ */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    while (1)
      ;
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    while (1)
      ;
  }
}

/* ------------------------------------------------------------------ */
void GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /* ── Motor driver ────────────────────────────────────────────── */
  motor_driver.motor_left.IN1_PORT = GPIOA;
  motor_driver.motor_left.IN1_PIN = GPIO_PIN_0;
  motor_driver.motor_left.IN2_PORT = GPIOA;
  motor_driver.motor_left.IN2_PIN = GPIO_PIN_1;
  motor_driver.motor_right.IN1_PORT = GPIOA;
  motor_driver.motor_right.IN1_PIN = GPIO_PIN_2;
  motor_driver.motor_right.IN2_PORT = GPIOA;
  motor_driver.motor_right.IN2_PIN = GPIO_PIN_3;

  /* ── Ultrasonic sensors — TRIG pins (plain GPIO output) ───────
   * ECHO pins (PA6, PA7, PB0) are configured as AF input captures
   * inside TIM_Init; do NOT re-init them as plain GPIO here.       */
  ultrasonic_front.TRIG_PORT = GPIOB;
  ultrasonic_front.TRIG_PIN = GPIO_PIN_8;
  ultrasonic_front.ECHO_PORT = GPIOA;
  ultrasonic_front.ECHO_PIN = GPIO_PIN_6; // TIM3_CH1

  ultrasonic_left.TRIG_PORT = GPIOB;
  ultrasonic_left.TRIG_PIN = GPIO_PIN_9;
  ultrasonic_left.ECHO_PORT = GPIOA;
  ultrasonic_left.ECHO_PIN = GPIO_PIN_7; // TIM3_CH2

  ultrasonic_right.TRIG_PORT = GPIOB;
  ultrasonic_right.TRIG_PIN = GPIO_PIN_10;
  ultrasonic_right.ECHO_PORT = GPIOB;
  ultrasonic_right.ECHO_PIN = GPIO_PIN_0; // TIM3_CH3

  /* ── IR sensors ──────────────────────────────────────────────── */
  ir_sensor_left.GPIO_PORT = GPIOC;
  ir_sensor_left.GPIO_PIN = GPIO_PIN_0;
  ir_sensor_left.mode = IR_MODE_DIGITAL;
  ir_sensor_right.GPIO_PORT = GPIOC;
  ir_sensor_right.GPIO_PIN = GPIO_PIN_1;
  ir_sensor_right.mode = IR_MODE_DIGITAL;

  /* ── Onboard LED (PC13) ──────────────────────────────────────── */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* ------------------------------------------------------------------ */
void TIM_Init(void) {
  static TIM_HandleTypeDef htim3;
  TIM_IC_InitTypeDef sConfigIC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64 - 1; // 64 MHz → 1 MHz (1 µs / tick)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim3);

  /* Common IC config — detect both rising and falling edges */
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  /* CH1 → PA6 → Front sensor */
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
  ultrasonic_front.htim = &htim3;
  ultrasonic_front.tim_channel = TIM_CHANNEL_1;

  /* CH2 → PA7 → Left sensor */
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);
  ultrasonic_left.htim = &htim3;
  ultrasonic_left.tim_channel = TIM_CHANNEL_2;

  /* CH3 → PB0 → Right sensor */
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
  ultrasonic_right.htim = &htim3;
  ultrasonic_right.tim_channel = TIM_CHANNEL_3;

  HAL_TIM_Base_Start(&htim3);

  /* Start IC on all three channels */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  motor_driver.motor_right.PWM_Max = 1000;
}

/* ------------------------------------------------------------------ */
void I2C_Init(void) { imu.i2c_addr = MPU6050_ADDR; }
void ADC_Init(void) { /* Configure via CubeMX if needed */ }

/* ------------------------------------------------------------------ */
/**
 * @brief  Dispatch each channel's echo event to the matching sensor.
 *
 * htim->Channel holds the active channel as HAL_TIM_ACTIVE_CHANNEL_x,
 * so we can route without touching any global state.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance != TIM3)
    return;

  switch (htim->Channel) {
  case HAL_TIM_ACTIVE_CHANNEL_1:
    HC_SR04_Echo_Callback(&ultrasonic_front);
    break;
  case HAL_TIM_ACTIVE_CHANNEL_2:
    HC_SR04_Echo_Callback(&ultrasonic_left);
    break;
  case HAL_TIM_ACTIVE_CHANNEL_3:
    HC_SR04_Echo_Callback(&ultrasonic_right);
    break;
  default:
    break;
  }
}

/* ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_0) {        /* left IR  */
  } else if (GPIO_Pin == GPIO_PIN_1) { /* right IR */
  }
}

/* ------------------------------------------------------------------ */
void TIM3_IRQHandler(void) {
  /* Any of the three sensors can trigger this — HAL sorts it out */
  HAL_TIM_IRQHandler(ultrasonic_front.htim);
}