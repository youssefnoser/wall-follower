#include "esp8266.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "wall_follower_fsm.h"
#include "wifi_control.h"
#include <stdio.h>

/* Uncomment to build in WiFi remote-control mode instead of wall follower. */
/* #define WIFI_CONTROL_MODE */
#define WIFI_CONTROL_MODE
#define WIFI_SSID     "DIRECT-NS-Hotspot"
#define WIFI_PASSWORD "12345678"

/* Global Handlers */
L298N_t motor_driver;

HC_SR04_t ultrasonic_front;
HC_SR04_t ultrasonic_left;
HC_SR04_t ultrasonic_right;

IR_Sensor_t ir_sensor_front;
MPU6050_t imu;

/* FSM context */
WallFollower_t fsm;

/* Forward Declarations */
extern void initialise_monitor_handles(void);
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void I2C_Init(void);
void ADC_Init(void);

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

  IR_Sensor_Init(&ir_sensor_front);

  L298N_Init(&motor_driver);

  // MPU6050_Init(&imu);

#ifdef WIFI_CONTROL_MODE

  WiFiControl_Init(WIFI_SSID, WIFI_PASSWORD);

  while (1) {
    WiFiControl_Update(&motor_driver, &ultrasonic_front, &ultrasonic_left,
                       &ultrasonic_right, &imu);
  }

#else /* wall follower mode */

  /* Initialise FSM – must be called after all hardware inits */
  WallFollower_Init(&fsm);

  printf("Wall Follower FSM ready.\n");

  while (1) {
    /* Blink LED every 2000 ms (non-blocking) */
    static uint32_t led_last = 0;
    uint32_t now = HAL_GetTick();
    if ((now - led_last) >= 2000) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      led_last = now;
    }

    /* ---- Ultrasonics ------------------------------------------------- */
    HC_SR04_StartMeasurement(&ultrasonic_front);
    HAL_Delay(50);
    HC_SR04_StartMeasurement(&ultrasonic_left);
    HAL_Delay(50);
    HC_SR04_StartMeasurement(&ultrasonic_right);
    HAL_Delay(50);

    float dist_front = HC_SR04_GetDistance(&ultrasonic_front);
    float dist_left = HC_SR04_GetDistance(&ultrasonic_left);
    float dist_right = HC_SR04_GetDistance(&ultrasonic_right);

    /* ---- IR sensors -------------------------------------------------- */
    /*
     * Front IR is used to detect close obstacles as a failsafe for ultrasonics.
     * IR_NO_OBJECT       → route clear, safe to move
     * IR_OBJECT_DETECTED → obstacle very close, stop!
     */
    uint8_t ir_safe = (IR_Sensor_GetState(&ir_sensor_front) == IR_NO_OBJECT);

    /* ---- IMU --------------------------------------------------------- */
    MPU6050_ReadAll(&imu);

    /* ---- FSM step ----------------------------------------------------- */
    WallFollower_Update(&fsm, dist_front, dist_left, dist_right, ir_safe,
                        imu.gyro.z); /* gyro Z in degrees/second */

    /* ---- Debug print -------------------------------------------------- */
    printf("[FSM] %-13s | F:%5.1f L:%5.1f R:%5.1f cm | IR:%d | Yaw:%7.1f deg | "
           "Retries "
           "L:%d R:%d\n",
           FSM_StateName(fsm.state), dist_front, dist_left, dist_right, ir_safe,
           fsm.yaw, fsm.retry_left, fsm.retry_right);

    HAL_Delay(100);
  }

#endif /* WIFI_CONTROL_MODE */

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

  motor_driver.motor_left.IN1_PORT = GPIOB;
  motor_driver.motor_left.IN1_PIN = GPIO_PIN_12;
  motor_driver.motor_left.IN2_PORT = GPIOB;
  motor_driver.motor_left.IN2_PIN = GPIO_PIN_13;
  motor_driver.motor_right.IN1_PORT = GPIOB;
  motor_driver.motor_right.IN1_PIN = GPIO_PIN_14;
  motor_driver.motor_right.IN2_PORT = GPIOB;
  motor_driver.motor_right.IN2_PIN = GPIO_PIN_15;

  motor_driver.motor_left.PWM_Channel = TIM_CHANNEL_1;  /* PA0 */
  motor_driver.motor_right.PWM_Channel = TIM_CHANNEL_2; /* PA1 */

  ultrasonic_front.TRIG_PORT = GPIOB;
  ultrasonic_front.TRIG_PIN = GPIO_PIN_8;
  ultrasonic_front.ECHO_PORT = GPIOA;
  ultrasonic_front.ECHO_PIN = GPIO_PIN_6; /* TIM3_CH1 */

  ultrasonic_left.TRIG_PORT = GPIOA;
  ultrasonic_left.TRIG_PIN = GPIO_PIN_7;
  ultrasonic_left.ECHO_PORT = GPIOB;
  ultrasonic_left.ECHO_PIN = GPIO_PIN_9; /* TIM4_CH4 */

  ultrasonic_right.TRIG_PORT = GPIOB;
  ultrasonic_right.TRIG_PIN = GPIO_PIN_10;
  ultrasonic_right.ECHO_PORT = GPIOB;
  ultrasonic_right.ECHO_PIN = GPIO_PIN_0; /* TIM3_CH3 */

  ir_sensor_front.GPIO_PORT = GPIOA;
  ir_sensor_front.GPIO_PIN = GPIO_PIN_4;
  ir_sensor_front.mode = IR_MODE_DIGITAL;

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
  static TIM_HandleTypeDef htim4;
  TIM_IC_InitTypeDef sConfigIC = {0};

  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim3);

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64 - 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xFFFF;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim4);

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1);
  ultrasonic_front.htim = &htim3;
  ultrasonic_front.tim_channel = TIM_CHANNEL_1;

  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
  ultrasonic_left.htim = &htim4;
  ultrasonic_left.tim_channel = TIM_CHANNEL_4;

  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
  ultrasonic_right.htim = &htim3;
  ultrasonic_right.tim_channel = TIM_CHANNEL_3;

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);

  motor_driver.motor_left.PWM_Max = 1000;
  motor_driver.motor_right.PWM_Max = 1000;
}

/* ------------------------------------------------------------------ */
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

void ADC_Init(void) {}

/* ------------------------------------------------------------------ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      HC_SR04_Echo_Callback(&ultrasonic_front);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      HC_SR04_Echo_Callback(&ultrasonic_right);
      break;
    default:
      break;
    }
  } else if (htim->Instance == TIM4) {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
      HC_SR04_Echo_Callback(&ultrasonic_left);
  }
}

/* ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_4) { /* front IR – handled by FSM polling */
  }
}

/* ------------------------------------------------------------------ */
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_front.htim); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_left.htim); }
