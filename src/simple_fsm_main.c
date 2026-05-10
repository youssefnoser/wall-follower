

#include "dwt_stm32_delay.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "mpu6050.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>

/* ---- Left-wall follower tuning --------------------------------------- */
#define LEFT_WALL_SETPOINT  15.0f  /* cm – target distance from left wall  */
#define LEFT_WALL_KP         2.0f
#define LEFT_WALL_KI         0.0f
#define LEFT_WALL_KD         0.5f
#define LEFT_BASE_SPEED        35  /* 0-100 PWM base duty                  */
#define LEFT_EMA_ALPHA        0.3f /* ultrasonic EMA filter weight          */
#define FRONT_OBSTACLE_CM    20.0f /* cm – front obstacle stop threshold    */

/* Global Handlers */
L298N_t motor_driver;

HC_SR04_t ultrasonic_front;
HC_SR04_t ultrasonic_left;
HC_SR04_t ultrasonic_right;

IR_Sensor_t ir_sensor_front;
MPU6050_t imu;

/* Forward Declarations */
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void I2C_Init(void);
void ADC_Init(void);

extern void initialise_monitor_handles(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
    
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

  MPU6050_Init(&imu);

  /* Zero biases by default */
  imu.accel_bias.x = imu.accel_bias.y = imu.accel_bias.z = 0.0f;
  imu.gyro_bias.x = imu.gyro_bias.y = imu.gyro_bias.z = 0.0f;

  /* Initialize filter params and state */
  imu.filt_alpha = 0.2f; /* weight for new samples (tunable) */
  imu.filt_initialized = 0;
  imu.accel_filt.x = imu.accel_filt.y = imu.accel_filt.z = 0.0f;
  imu.gyro_filt.x = imu.gyro_filt.y = imu.gyro_filt.z = 0.0f;

  /* If front IR sensor is covered at startup, enter IMU calibration mode.
     Cover the front IR (IR_OBJECT_DETECTED) to trigger calibration. */
  uint8_t do_calibration = 1;
     if (do_calibration)
  {
    /* Simple calibration: average N samples while stationary */
    #define IMU_CAL_SAMPLES 10000
    Vector3_t a_sum = {0}, g_sum = {0};
    for (int i = 0; i < IMU_CAL_SAMPLES; ++i)
    {
      MPU6050_ReadAll(&imu);
      a_sum.x += imu.accel.x;
      a_sum.y += imu.accel.y;
      a_sum.z += imu.accel.z;
      g_sum.x += imu.gyro.x;
      g_sum.y += imu.gyro.y;
      g_sum.z += imu.gyro.z;
      HAL_Delay(10);
    }
    imu.accel_bias.x = a_sum.x / IMU_CAL_SAMPLES;
    imu.accel_bias.y = a_sum.y / IMU_CAL_SAMPLES;
    /* For Z, subtract expected +1g due to gravity when stationary and oriented so Z faces up */
    imu.accel_bias.z = (a_sum.z / IMU_CAL_SAMPLES) - 1.0f;

    imu.gyro_bias.x = g_sum.x / IMU_CAL_SAMPLES;
    imu.gyro_bias.y = g_sum.y / IMU_CAL_SAMPLES;
    imu.gyro_bias.z = g_sum.z / IMU_CAL_SAMPLES;

    printf("IMU Calibration complete. Accel bias [%.3f, %.3f, %.3f] g  Gyro bias [%.3f, %.3f, %.3f] dps\r\n",
           imu.accel_bias.x, imu.accel_bias.y, imu.accel_bias.z,
           imu.gyro_bias.x, imu.gyro_bias.y, imu.gyro_bias.z);
  }

  /* Seed filters from current sensor state so filtering doesn't start from zero. */
  MPU6050_ReadAll(&imu);
  MPU6050_ReadTemperature(&imu);
  imu.accel_filt.x = imu.accel.x - imu.accel_bias.x;
  imu.accel_filt.y = imu.accel.y - imu.accel_bias.y;
  imu.accel_filt.z = imu.accel.z - imu.accel_bias.z;
  imu.gyro_filt.x = imu.gyro.x - imu.gyro_bias.x;
  imu.gyro_filt.y = imu.gyro.y - imu.gyro_bias.y;
  imu.gyro_filt.z = imu.gyro.z - imu.gyro_bias.z;
  imu.filt_initialized = 1;

  /* Left-wall PID controller */
  PID_t pid_left_wall;
  PID_Init(&pid_left_wall, LEFT_WALL_KP, LEFT_WALL_KI, LEFT_WALL_KD,
           -LEFT_BASE_SPEED, LEFT_BASE_SPEED);

  float filt_left = 100.0f; /* EMA state, seeded high so first read converges */
  uint32_t last_pid_tick = HAL_GetTick();

  /* IMU telemetry timing (ms) */
  uint32_t imu_last = 0;
  const uint32_t imu_period = 200; /* 200 ms -> 5 Hz */

  // printf("Simple FSM ready - Moving forward (Stand-alone).\n");

  while (1)
  {
    /* Blink LED every 2000 ms to show it's alive */
    static uint32_t led_last = 0;
    uint32_t now = HAL_GetTick();
    if ((now - led_last) >= 2000)
    {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      led_last = now;
    }

    /* ---- Start Ultrasonic Measurements -------------------------------- */
    /* Start measurements sequentially to avoid cross-talk, with reduced delays */
    HC_SR04_StartMeasurement(&ultrasonic_front);
    HAL_Delay(40); /* Wait for front echo (max ~38ms for timeout) */
    float dist_front = HC_SR04_GetDistance(&ultrasonic_front);

    HC_SR04_StartMeasurement(&ultrasonic_left);
    HAL_Delay(40); /* Wait for left echo */
    float dist_left = HC_SR04_GetDistance(&ultrasonic_left);

    /* ---- Measure IR sensor -------------------------------------------- */
    /* IR_NO_OBJECT -> route clear, IR_OBJECT_DETECTED -> obstacle */
    uint8_t ir_safe = (IR_Sensor_GetState(&ir_sensor_front) == IR_NO_OBJECT);
    ir_safe = 1; /* For testing without IR sensor */
    /* ---- EMA filter on left ultrasonic -------------------------------- */
    filt_left = LEFT_EMA_ALPHA * dist_left + (1.0f - LEFT_EMA_ALPHA) * filt_left;

    /* ---- dt for PID --------------------------------------------------- */
    uint32_t pid_now = HAL_GetTick();
    float dt = (float)(pid_now - last_pid_tick) * 0.001f; /* ms → s */
    last_pid_tick = pid_now;

    /* ---- Logic: Move forward unless obstacle is detected -------------- */
    if (dist_front < FRONT_OBSTACLE_CM || !ir_safe)
    {
        /* Obstacle detected: stop and reset integral to avoid windup */
        L298N_StopAll(&motor_driver);
        PID_Reset(&pid_left_wall);
        printf("[FSM] STOP  | Front: %.1f cm  IR: %d\r\n", dist_front, ir_safe);
    }
    else
    {
        /* Left-wall PID: error = setpoint - measurement
         *   positive → too far from wall → steer left (right faster)
         *   negative → too close to wall → steer right (left faster)  */
        float pid_out = PID_Compute(&pid_left_wall,
                                    LEFT_WALL_SETPOINT, filt_left, dt);

        int speed_l = LEFT_BASE_SPEED - (int)pid_out;
        int speed_r = LEFT_BASE_SPEED + (int)pid_out;

        if (speed_l > 100) speed_l = 100;
        if (speed_l < 0)   speed_l = 0;
        if (speed_r > 100) speed_r = 100;
        if (speed_r < 0)   speed_r = 0;

        L298N_Move(&motor_driver, MOTOR_LEFT,  MOTOR_FORWARD, (uint8_t)speed_l);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, (uint8_t)speed_r);

        printf("[FSM] FWD   | Front: %.1f  Left(raw): %.1f  Left(filt): %.1f  PID: %.2f  L:%d R:%d\r\n",
               dist_front, dist_left, filt_left, pid_out, speed_l, speed_r);
    }

    /* IMU: periodic read and print (throttled) */
    // if ((now - imu_last) >= imu_period)
    // {
            MPU6050_ReadAll(&imu);
            MPU6050_ReadTemperature(&imu);

            /* Apply calibration biases */
            float ax = imu.accel.x - imu.accel_bias.x;
            float ay = imu.accel.y - imu.accel_bias.y;
            float az = imu.accel.z - imu.accel_bias.z;
            float gx = imu.gyro.x - imu.gyro_bias.x;
            float gy = imu.gyro.y - imu.gyro_bias.y;
            float gz = imu.gyro.z - imu.gyro_bias.z;

            /* Exponential Moving Average filter (EMA):
               new_filt = alpha*sample + (1-alpha)*prev_filt
               alpha is stored in imu.filt_alpha. */
            if (!imu.filt_initialized)
            {
              imu.accel_filt.x = ax;
              imu.accel_filt.y = ay;
              imu.accel_filt.z = az;
              imu.gyro_filt.x = gx;
              imu.gyro_filt.y = gy;
              imu.gyro_filt.z = gz;
              imu.filt_initialized = 1;
            }
            else
            {
              float a = imu.filt_alpha;
              float b = 1.0f - a;
              imu.accel_filt.x = a * ax + b * imu.accel_filt.x;
              imu.accel_filt.y = a * ay + b * imu.accel_filt.y;
              imu.accel_filt.z = a * az + b * imu.accel_filt.z;
              imu.gyro_filt.x = a * gx + b * imu.gyro_filt.x;
              imu.gyro_filt.y = a * gy + b * imu.gyro_filt.y;
              imu.gyro_filt.z = a * gz + b * imu.gyro_filt.z;
            }

            printf("IMU (filt): Accel [%.3f, %.3f, %.3f] g  Gyro [%.3f, %.3f, %.3f] dps  Temp: %.2f C\r\n",
              imu.accel_filt.x, imu.accel_filt.y, imu.accel_filt.z,
              imu.gyro_filt.x, imu.gyro_filt.y, imu.gyro_filt.z,
              imu.temperature);
      imu_last = now;
    // }

    /* We removed the 50ms loop delay here because the 80ms delay above is enough for the loop rate */
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
void GPIO_Init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  motor_driver.motor_left.IN2_PORT = GPIOB;
  motor_driver.motor_left.IN2_PIN = GPIO_PIN_12;
  motor_driver.motor_left.IN1_PORT = GPIOB;
  motor_driver.motor_left.IN1_PIN = GPIO_PIN_13;
  motor_driver.motor_right.IN2_PORT = GPIOB;
  motor_driver.motor_right.IN2_PIN = GPIO_PIN_14;
  motor_driver.motor_right.IN1_PORT = GPIOB;
  motor_driver.motor_right.IN1_PIN = GPIO_PIN_15;

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
void TIM_Init(void)
{
  static TIM_HandleTypeDef htim2;
  static TIM_HandleTypeDef htim3;
  static TIM_HandleTypeDef htim4;
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  /* PA0 = TIM2_CH1 (motor_left EN), PA1 = TIM2_CH2 (motor_right EN) */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode  = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64 - 1;     /* 64 MHz / 64 = 1 MHz tick */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1;      /* 1 kHz PWM, matches PWM_Max=1000 */
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim2);

  sConfigOC.OCMode     = TIM_OCMODE_PWM1;
  sConfigOC.Pulse      = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

  motor_driver.motor_left.htim  = &htim2;
  motor_driver.motor_right.htim = &htim2;

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

void ADC_Init(void) {}

/* ------------------------------------------------------------------ */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3)
  {
    switch (htim->Channel)
    {
    case HAL_TIM_ACTIVE_CHANNEL_1:
      HC_SR04_Echo_Callback(&ultrasonic_front);
      break;
    case HAL_TIM_ACTIVE_CHANNEL_3:
      HC_SR04_Echo_Callback(&ultrasonic_right);
      break;
    default:
      break;
    }
  }
  else if (htim->Instance == TIM4)
  {
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
      HC_SR04_Echo_Callback(&ultrasonic_left);
  }
}

/* ------------------------------------------------------------------ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4)
  { /* front IR – handled by FSM polling */
  }
}

/* ------------------------------------------------------------------ */
void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_front.htim); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_left.htim); }
