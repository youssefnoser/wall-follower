#include "esp8266.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/* Global Handlers */
L298N_t motor_driver;

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

  /* ── ESP8266 ─────────────────────────────────────────────────── */
  ESP8266_Init();
  ESP8266_SendCmd("AT");
  if (ESP8266_CheckResponse("OK", 10000))
    printf("ESP8266 AT Command OK\n");

  if (ESP8266_ConnectWiFi("Youssef's A51", "lol123a@@"))
    printf("Successfully connected to WiFi!\n");
  else
    printf("Failed to connect to WiFi.\n");

  while (1) {
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

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

/* ------------------------------------------------------------------ */
void GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  motor_driver.motor_left.IN1_PORT = GPIOA;
  motor_driver.motor_left.IN1_PIN = GPIO_PIN_0;
  motor_driver.motor_left.IN2_PORT = GPIOA;
  motor_driver.motor_left.IN2_PIN = GPIO_PIN_1;
  motor_driver.motor_right.IN1_PORT = GPIOA;
  motor_driver.motor_right.IN1_PIN = GPIO_PIN_2;
  motor_driver.motor_right.IN2_PORT = GPIOA;
  motor_driver.motor_right.IN2_PIN = GPIO_PIN_3;

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

  ir_sensor_left.GPIO_PORT = GPIOC;
  ir_sensor_left.GPIO_PIN = GPIO_PIN_0;
  ir_sensor_left.mode = IR_MODE_DIGITAL;
  ir_sensor_right.GPIO_PORT = GPIOC;
  ir_sensor_right.GPIO_PIN = GPIO_PIN_1;
  ir_sensor_right.mode = IR_MODE_DIGITAL;

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
  htim3.Init.Prescaler = 64 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim3);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
  ultrasonic_front.htim = &htim3;
  ultrasonic_front.tim_channel = TIM_CHANNEL_1;

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2);
  ultrasonic_left.htim = &htim3;
  ultrasonic_left.tim_channel = TIM_CHANNEL_2;

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
  ultrasonic_right.htim = &htim3;
  ultrasonic_right.tim_channel = TIM_CHANNEL_3;

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  motor_driver.motor_right.PWM_Max = 1000;
}

/* ------------------------------------------------------------------ */
void I2C_Init(void) { imu.i2c_addr = MPU6050_ADDR; }
void ADC_Init(void) {}

/* ------------------------------------------------------------------ */
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
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_front.htim); }