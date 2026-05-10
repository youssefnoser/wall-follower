

#include "ESP8266.h"
#include "dwt_stm32_delay.h"
#include "flash_logger.h"
#include "esp_ring.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include "wifi_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Left-wall follower tuning --------------------------------------- */
#define LEFT_WALL_SETPOINT  15.0f  /* cm – initial target distance (runtime-tunable) */
#define LEFT_WALL_KP         2.0f
#define LEFT_WALL_KI         0.0f
#define LEFT_WALL_KD         0.5f
#define LEFT_BASE_SPEED        35  /* 0-100 PWM base duty                  */
#define LEFT_EMA_ALPHA        0.3f /* ultrasonic EMA filter weight          */
#define FRONT_OBSTACLE_CM    20.0f /* cm – front obstacle stop threshold    */

/* ---- WiFi / PID-tuner ------------------------------------------------ */
#define WIFI_SSID     "Yousef"
#define WIFI_PASSWORD "20202020"

/* Global Handlers */
UART_HandleTypeDef huart1; /* ESP8266 UART handle – aliased by _WIFI_USART macro */
L298N_t motor_driver;

HC_SR04_t ultrasonic_front;
HC_SR04_t ultrasonic_left;
HC_SR04_t ultrasonic_right;

IR_Sensor_t ir_sensor_front;

/* Forward Declarations */
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void ADC_Init(void);
void USART1_UART_Init(void);

/* -----------------------------------------------------------------------
 * WiFi PID tuner
 *
 * Accepted TCP commands (send from phone/PC to port 1234):
 *   PID=Kp,Ki,Kd   – update all three gains and reset integral
 *   SP=15.0         – change wall setpoint distance (cm)
 *   GET             – reply with current Kp, Ki, Kd, SP
 *
 * The MCU replies with a one-line ACK so you can confirm the change.
 * --------------------------------------------------------------------- */
static int8_t wifi_link = -1;

static void pid_poll_wifi(PID_t *pid, float *setpoint) {
  char snap[128];
  uint16_t len = esp_ring_urc_peek((uint8_t *)snap, sizeof(snap) - 1);
  if (len == 0) return;
  snap[len] = '\0';

  uint16_t pos = 0;
  while (pos < len) {
    /* +IPD,<id>,<n>:<payload> */
    if (strncmp(snap + pos, "+IPD,", 5) == 0) {
      const char *colon = strchr(snap + pos, ':');
      if (!colon) break;
      int id = 0, n = 0;
      if (sscanf(snap + pos, "+IPD,%d,%d:", &id, &n) == 2 && n > 0) {
        uint16_t hdr = (uint16_t)(colon - (snap + pos)) + 1;
        if (pos + hdr + (uint16_t)n > len) break;
        char payload[64];
        uint16_t cp = ((uint16_t)n < (uint16_t)(sizeof(payload) - 1))
                          ? (uint16_t)n : (uint16_t)(sizeof(payload) - 1);
        memcpy(payload, snap + pos + hdr, cp);
        payload[cp] = '\0';
        while (cp > 0 && (payload[cp - 1] == '\r' || payload[cp - 1] == '\n'))
          payload[--cp] = '\0';
        wifi_link = (int8_t)id;

        char ack[80];
        int alen = 0;

        if (strncmp(payload, "PID=", 4) == 0) {
          /* PID=Kp,Ki,Kd */
          char *p = payload + 4;
          char *end;
          float kp = strtof(p, &end);
          if (*end == ',') {
            float ki = strtof(end + 1, &end);
            if (*end == ',') {
              float kd = strtof(end + 1, NULL);
              PID_Init(pid, kp, ki, kd, -LEFT_BASE_SPEED, LEFT_BASE_SPEED);
              PID_Reset(pid);
              alen = snprintf(ack, sizeof(ack),
                              "OK PID KP=%.3f KI=%.3f KD=%.3f\r\n", kp, ki, kd);
              printf("[PID tuner] KP=%.3f KI=%.3f KD=%.3f\r\n", kp, ki, kd);
            }
          }
        } else if (strncmp(payload, "SP=", 3) == 0) {
          /* SP=<cm>  – change wall setpoint */
          float sp = strtof(payload + 3, NULL);
          *setpoint = sp;
          alen = snprintf(ack, sizeof(ack), "OK SP=%.1f\r\n", sp);
          printf("[PID tuner] Setpoint=%.1f cm\r\n", sp);
        } else if (strncmp(payload, "GET", 3) == 0) {
          /* GET – query current params */
          alen = snprintf(ack, sizeof(ack),
                          "KP=%.3f KI=%.3f KD=%.3f SP=%.1f\r\n",
                          pid->Kp, pid->Ki, pid->Kd, *setpoint);
        }

        if (alen > 0)
          Wifi_TcpIp_SendDataTcp((uint8_t)wifi_link, (uint16_t)alen,
                                 (uint8_t *)ack);
        pos += hdr + (uint16_t)n;
        continue;
      }
      pos += 5;
      continue;
    }
    /* <id>,CONNECT / <id>,CLOSED – skip to end of line */
    if (snap[pos] >= '0' && snap[pos] <= '4' && (pos + 1 < len) &&
        snap[pos + 1] == ',') {
      const char *eol = strchr(snap + pos, '\n');
      if (!eol) break;
      pos += (uint16_t)(eol - (snap + pos)) + 1;
      continue;
    }
    pos++;
  }
  if (pos > 0) esp_ring_urc_consume(pos);
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
  flash_logger_init();

  GPIO_Init();
  TIM_Init();
  ADC_Init();

  HC_SR04_Init(&ultrasonic_front);
  HC_SR04_Init(&ultrasonic_left);
  HC_SR04_Init(&ultrasonic_right);

  IR_Sensor_Init(&ir_sensor_front);

  L298N_Init(&motor_driver);

  USART1_UART_Init();
  WiFiControl_Init(WIFI_SSID, WIFI_PASSWORD);

  /* Left-wall PID controller (gains and setpoint tunable via WiFi at runtime) */
  PID_t pid_left_wall;

  PID_Init(&pid_left_wall, LEFT_WALL_KP, LEFT_WALL_KI, LEFT_WALL_KD,
           -LEFT_BASE_SPEED, LEFT_BASE_SPEED);
  float wall_setpoint = LEFT_WALL_SETPOINT; /* runtime-mutable via "SP=" cmd */

  float filt_left = 100.0f; /* EMA state, seeded high so first read converges */
  uint32_t last_pid_tick = HAL_GetTick();

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
    /* ---- Poll WiFi for PID tuning commands ---------------------------- */
    pid_poll_wifi(&pid_left_wall, &wall_setpoint);

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
                                    wall_setpoint, filt_left, dt);

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

    /* Loop rate is ~80ms from the two 40ms sensor delays above */
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

/* ------------------------------------------------------------------ */
/* USART1: PA9 TX (AF push-pull) / PA10 RX (input pull-up), 115200-8-N-1.
 * Used by the ESP8266 driver. */
void USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin   = GPIO_PIN_9;
  GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  _WIFI_USART.Instance                    = USART1;
  _WIFI_USART.Init.BaudRate               = 115200;
  _WIFI_USART.Init.WordLength             = UART_WORDLENGTH_8B;
  _WIFI_USART.Init.StopBits              = UART_STOPBITS_1;
  _WIFI_USART.Init.Parity                = UART_PARITY_NONE;
  _WIFI_USART.Init.Mode                  = UART_MODE_TX_RX;
  _WIFI_USART.Init.HwFlowCtl            = UART_HWCONTROL_NONE;
  _WIFI_USART.Init.OverSampling         = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&_WIFI_USART) != HAL_OK)
    while (1);

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&_WIFI_USART); }

/* HAL calls this after each single-byte RX IT completes.
 * Wifi_RxCallBack() pushes the byte into the ring and re-arms the next IT. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
    Wifi_RxCallBack();
}
