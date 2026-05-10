#include "ESP8266.h"
#include "dwt_stm32_delay.h"
#include "esp_ring.h"
#include "flash_logger.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "pid.h"
#include "stm32f1xx_hal.h"
#include "wifi_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Corridor-centering tuning --------------------------------------- */
#define CENTER_KP                                                              \
  0.5f /* right motor inverted in firmware; corrected pid_out direction */
#define CENTER_KI 0.03f /* small I-term to offset steady-state edge bias */
#define CENTER_KD                                                              \
  0.2f /* derivative damping; increase if oscillation appears                  \
        */
/*           if oscillation appears after Kp increase                   */
#define CENTER_MARGIN_CM                                                       \
  2.0f /* was 5.0 – 5 cm dead-band ate 30% of correction range in a    */
       /*           40-60 cm corridor; 2 cm is appropriate here         */
#define BASE_SPEED                                                             \
  42 /* was 35  – more headroom for differential steering; avoids     */
     /*           stiction when one motor is corrected toward zero    */
#define EMA_ALPHA                                                              \
  0.45f /* was 0.3 – cuts EMA lag from ~340 ms to ~195 ms at 120 ms     */
        /*           loop rate; faster sensor tracking, still filters    */
        /*           ultrasonic noise                                    */
#define FRONT_OBSTACLE_CM                                                      \
  28.0f /* was 20 – gives 2 extra loop iterations of reaction time     */
        /*          at BASE_SPEED 42 before impact                     */

/* Nonlinear error compression: small errors linear, large errors smoothly
 * saturate. Tune to ~half corridor width. */
#define SENSITIVITY 20.0f
/* Speed floor used by error-coupled base-speed blending near walls. */
#define SPEED_FLOOR 22

/* Per-side multiplicative PWM scale factors (float, positive).
 *
 * Purpose: compensate for physical motor speed mismatch without an additive
 * offset that interacts badly with PID output at low duty cycles.
 *
 * CALIBRATE:
 *   1. Send  SPEED=50  and  AUTO  via WiFi so PID runs at a known base duty.
 *   2. Drive in open space and observe which side lags.
 *   3. Send  LFAC=1.08  (or RFAC=1.08) to scale up the slow side.
 *      Typical range: 0.80 – 1.20.  Values outside that suggest a mechanical
 *      problem (friction, misaligned wheel) rather than a motor mismatch.
 *   4. Repeat until the robot tracks straight, then restore corridor-centering.
 *
 * Interaction with PID:
 *   speed_l = (base_speed + pid_out) * left_factor
 *   speed_r = (base_speed - pid_out) * right_factor
 *   Both are clamped to [0, 100] after scaling.
 */
#define LEFT_MOTOR_FACTOR 1.0f
#define RIGHT_MOTOR_FACTOR 1.0f

/* ---- WiFi / PID-tuner ------------------------------------------------ */
#define WIFI_SSID "Youssef's A51"
#define WIFI_PASSWORD "lol123a@@@"

/* Global Handlers */
UART_HandleTypeDef
    huart1; /* ESP8266 UART handle – aliased by _WIFI_USART macro */
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
 * WiFi corridor-centering tuner
 *
 * TCP commands (port 1234):
 *   PID=Kp,Ki,Kd   – update gains and reset integral
 *   MB=2.5          – set dead-band margin (cm)
 *   SPEED=<n>       – set base PWM duty 0-100
 *   LFAC=<f>        – left motor scale factor, e.g. 1.05 (range 0.5–2.0)
 *   RFAC=<f>        – right motor scale factor, e.g. 0.95
 *   GET             – reply with KP, KI, KD, MB, SPD, LFAC, RFAC, RUN, L, F, R
 *   DATA            – reply with just L, F, R sensor readings
 *   STOP / GO       – halt / resume autonomous loop
 *   AUTO            – switch to autonomous PID corridor-centering mode
 *   FWD/BWD/LEFT/RIGHT – manual drive commands
 * --------------------------------------------------------------------- */
static int8_t wifi_link = -1;
static float g_dist_front = 200.0f;
static float g_filt_left = 100.0f;
static float g_filt_right = 100.0f;
/* 0=AUTO (PID centering), 1=MANUAL (remote control) */
static uint8_t g_mode = 0;
/* manual motor command: 0=stop 1=fwd 2=bwd 3=left 4=right */
static uint8_t g_manual_cmd = 0;
/* runtime-tunable base PWM duty (0-100), default from #define */
static uint8_t g_base_speed = BASE_SPEED;
/* per-side multiplicative PWM scale factors – runtime-mutable via LFAC/RFAC */
static float g_left_factor = LEFT_MOTOR_FACTOR;
static float g_right_factor = RIGHT_MOTOR_FACTOR;

/* -----------------------------------------------------------------------
 * apply_factor – clamp a scaled duty to [0, 100]
 * --------------------------------------------------------------------- */
static inline uint8_t apply_factor(float raw, float factor) {
  int v = (int)(raw * factor);
  if (v > 100)
    v = 100;
  if (v < 0)
    v = 0;
  return (uint8_t)v;
}

static void pid_poll_wifi(PID_t *pid, float *margin, uint8_t *running) {
  char snap[128];
  uint16_t len = esp_ring_urc_peek((uint8_t *)snap, sizeof(snap) - 1);
  if (len == 0)
    return;
  snap[len] = '\0';

  uint16_t pos = 0;
  while (pos < len) {
    /* +IPD,<id>,<n>:<payload> */
    if (strncmp(snap + pos, "+IPD,", 5) == 0) {
      const char *colon = strchr(snap + pos, ':');
      if (!colon)
        break;
      int id = 0, n = 0;
      if (sscanf(snap + pos, "+IPD,%d,%d:", &id, &n) == 2 && n > 0) {
        uint16_t hdr = (uint16_t)(colon - (snap + pos)) + 1;
        if (pos + hdr + (uint16_t)n > len)
          break;
        char payload[64];
        uint16_t cp = ((uint16_t)n < (uint16_t)(sizeof(payload) - 1))
                          ? (uint16_t)n
                          : (uint16_t)(sizeof(payload) - 1);
        memcpy(payload, snap + pos + hdr, cp);
        payload[cp] = '\0';
        while (cp > 0 && (payload[cp - 1] == '\r' || payload[cp - 1] == '\n'))
          payload[--cp] = '\0';
        wifi_link = (int8_t)id;

        char ack[96];
        int alen = 0;

        if (strncmp(payload, "PID=", 4) == 0) {
          /* PID=Kp,Ki,Kd */
          char *p = payload + 4, *end;
          float kp = strtof(p, &end);
          if (*end == ',') {
            float ki = strtof(end + 1, &end);
            if (*end == ',') {
              float kd = strtof(end + 1, NULL);
              PID_Init(pid, kp, ki, kd, -BASE_SPEED, BASE_SPEED);
              PID_Reset(pid);
              alen = snprintf(ack, sizeof(ack),
                              "OK KP=%.3f KI=%.3f KD=%.3f\r\n", kp, ki, kd);
              printf("[tuner] KP=%.3f KI=%.3f KD=%.3f\r\n", kp, ki, kd);
            }
          }
        } else if (strncmp(payload, "MB=", 3) == 0) {
          float mb = strtof(payload + 3, NULL);
          if (mb >= 0.0f)
            *margin = mb;
          alen = snprintf(ack, sizeof(ack), "OK MB=%.2f\r\n", *margin);
          printf("[tuner] Margin=%.2f cm\r\n", *margin);

        } else if (strncmp(payload, "LFAC=", 5) == 0) {
          /* Left motor scale factor */
          float f = strtof(payload + 5, NULL);
          if (f > 0.0f) {
            g_left_factor = f;
            alen =
                snprintf(ack, sizeof(ack), "OK LFAC=%.3f\r\n", g_left_factor);
            printf("[tuner] left_factor=%.3f\r\n", g_left_factor);
          } else {
            alen = snprintf(ack, sizeof(ack), "ERR LFAC must be > 0\r\n");
          }

        } else if (strncmp(payload, "RFAC=", 5) == 0) {
          /* Right motor scale factor */
          float f = strtof(payload + 5, NULL);
          if (f > 0.0f) {
            g_right_factor = f;
            alen =
                snprintf(ack, sizeof(ack), "OK RFAC=%.3f\r\n", g_right_factor);
            printf("[tuner] right_factor=%.3f\r\n", g_right_factor);
          } else {
            alen = snprintf(ack, sizeof(ack), "ERR RFAC must be > 0\r\n");
          }

        } else if (strncmp(payload, "DATA", 4) == 0) {
          alen = snprintf(ack, sizeof(ack), "L=%.1f F=%.1f R=%.1f\r\n",
                          g_filt_left, g_dist_front, g_filt_right);

        } else if (strncmp(payload, "GET", 3) == 0) {
          alen = snprintf(ack, sizeof(ack),
                          "KP=%.3f KI=%.3f KD=%.3f MB=%.2f SPD=%d "
                          "LFAC=%.3f RFAC=%.3f RUN=%d L=%.1f F=%.1f R=%.1f\r\n",
                          pid->Kp, pid->Ki, pid->Kd, *margin, g_base_speed,
                          g_left_factor, g_right_factor, (int)*running,
                          g_filt_left, g_dist_front, g_filt_right);

        } else if (strncmp(payload, "STOP", 4) == 0) {
          *running = 0;
          g_manual_cmd = 0;
          alen = snprintf(ack, sizeof(ack), "OK STOPPED\r\n");
        } else if (strncmp(payload, "GO", 2) == 0) {
          *running = 1;
          alen = snprintf(ack, sizeof(ack), "OK RUNNING\r\n");
        } else if (strncmp(payload, "SPEED=", 6) == 0) {
          int spd = (int)strtof(payload + 6, NULL);
          if (spd < 0)
            spd = 0;
          if (spd > 100)
            spd = 100;
          g_base_speed = (uint8_t)spd;
          alen = snprintf(ack, sizeof(ack), "OK SPEED=%d\r\n", g_base_speed);
        } else if (strncmp(payload, "AUTO", 4) == 0) {
          g_mode = 0;
          *running = 1;
          alen = snprintf(ack, sizeof(ack), "OK AUTO\r\n");
        } else if (strncmp(payload, "FWD", 3) == 0) {
          g_mode = 1;
          g_manual_cmd = 1;
          *running = 1;
          alen = snprintf(ack, sizeof(ack), "OK FWD\r\n");
        } else if (strncmp(payload, "BWD", 3) == 0) {
          g_mode = 1;
          g_manual_cmd = 2;
          *running = 1;
          alen = snprintf(ack, sizeof(ack), "OK BWD\r\n");
        } else if (strncmp(payload, "LEFT", 4) == 0) {
          g_mode = 1;
          g_manual_cmd = 3;
          *running = 1;
          alen = snprintf(ack, sizeof(ack), "OK LEFT\r\n");
        } else if (strncmp(payload, "RIGHT", 5) == 0) {
          g_mode = 1;
          g_manual_cmd = 4;
          *running = 1;
          alen = snprintf(ack, sizeof(ack), "OK RIGHT\r\n");
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
      if (!eol)
        break;
      pos += (uint16_t)(eol - (snap + pos)) + 1;
      continue;
    }
    pos++;
  }
  if (pos > 0)
    esp_ring_urc_consume(pos);
}

int main(void) {
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

  /* Corridor-centering PID (all params tunable via WiFi at runtime) */
  PID_t pid_center;
  PID_Init(&pid_center, CENTER_KP, CENTER_KI, CENTER_KD, -BASE_SPEED,
           BASE_SPEED);
  float margin =
      CENTER_MARGIN_CM; /* dead-band, runtime-mutable via "MB=" cmd */

  uint32_t last_pid_tick = HAL_GetTick();
  uint8_t running = 1; /* STOP/GO command toggles this */

  while (1) {
    /* Blink LED every 2000 ms to show it's alive */
    static uint32_t led_last = 0;
    uint32_t now = HAL_GetTick();
    if ((now - led_last) >= 2000) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      led_last = now;
    }

    /* ---- Ultrasonic Measurements -------------------------------------- */
    /* Sequential firing avoids cross-talk; 40 ms covers the HC-SR04's     */
    /* worst-case 38 ms echo timeout for out-of-range targets.             */
    HC_SR04_StartMeasurement(&ultrasonic_front);
    HAL_Delay(40);
    g_dist_front = HC_SR04_GetDistance(&ultrasonic_front);

    HC_SR04_StartMeasurement(&ultrasonic_left);
    HAL_Delay(40);
    float dist_left = HC_SR04_GetDistance(&ultrasonic_left);

    HC_SR04_StartMeasurement(&ultrasonic_right);
    HAL_Delay(40);
    float dist_right = HC_SR04_GetDistance(&ultrasonic_right);

    /* ---- IR sensor ---------------------------------------------------- */
    uint8_t ir_safe = (IR_Sensor_GetState(&ir_sensor_front) == IR_NO_OBJECT);
    ir_safe = 1; /* For testing without IR sensor */

    /* ---- Poll WiFi for tuning commands -------------------------------- */
    pid_poll_wifi(&pid_center, &margin, &running);

    /* ---- EMA filters -------------------------------------------------- */
    /* α=0.45 gives ~195 ms time constant at the 120 ms loop rate.         */
    g_filt_left = EMA_ALPHA * dist_left + (1.0f - EMA_ALPHA) * g_filt_left;
    g_filt_right = EMA_ALPHA * dist_right + (1.0f - EMA_ALPHA) * g_filt_right;

    /* ---- dt for PID --------------------------------------------------- */
    uint32_t pid_now = HAL_GetTick();
    float dt = (float)(pid_now - last_pid_tick) * 0.001f;
    last_pid_tick = pid_now;

    /* ---- Control logic ------------------------------------------------ */
    if (!running) {
      L298N_StopAll(&motor_driver);

    } else if (g_mode == 1) {
      /* ---- Manual remote-control mode ---------------------------------- */
      /* Factors scale the base speed independently per side so manual      */
      /* straight-line driving also benefits from motor compensation.       */
      uint8_t ml = apply_factor((float)g_base_speed, g_left_factor);
      uint8_t mr = apply_factor((float)g_base_speed, g_right_factor);

      switch (g_manual_cmd) {
      case 1: /* FWD */
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, ml);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, mr);
        break;
      case 2: /* BWD */
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_BACKWARD, ml);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_BACKWARD, mr);
        break;
      case 3: /* LEFT – pivot: right fwd, left stop */
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, 0);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, mr);
        break;
      case 4: /* RIGHT – pivot: left fwd, right stop */
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, ml);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, 0);
        break;
      default:
        L298N_StopAll(&motor_driver);
        break;
      }

    } else if (g_dist_front < FRONT_OBSTACLE_CM || !ir_safe) {
      /* ---- Front obstacle stop ----------------------------------------- */
      L298N_StopAll(&motor_driver);
      PID_Reset(&pid_center);
      printf("[FSM] STOP  | F:%.1f L:%.1f R:%.1f\r\n", g_dist_front,
             g_filt_left, g_filt_right);

    } else {
      /* ---- AUTO corridor-centering PID --------------------------------- */
      /*
       * wall_error > 0 → robot closer to left wall  → steer right
       * wall_error < 0 → robot closer to right wall → steer left
       *
       * Per-side factors are applied AFTER the PID differential so that
       * the centering math stays symmetric.  The factor only compensates
       * for the physical speed difference at the wheel output.
       *
       *   raw_l = effective_base - pid_out → speed_l = raw_l * left_factor
       *   raw_r = effective_base + pid_out → speed_r = raw_r * right_factor
       */
      float wall_error = g_filt_left - g_filt_right;
      if (wall_error > -margin && wall_error < margin)
        wall_error = 0.0f;

      /* Speed-error coupling: large wall error lowers forward base speed,
       * reducing momentum and overshoot near walls. */
      float abs_err = fabsf(wall_error);
      float t = abs_err / SENSITIVITY;
      if (t > 1.0f)
        t = 1.0f;

      float effective_base = (float)g_base_speed * (1.0f - t) +
                             (float)SPEED_FLOOR * t;

      /* Nonlinear error compression: small errors pass through, large
       * errors smoothly saturate so PID output cannot grow unbounded. */
      float compressed = SENSITIVITY * tanhf(wall_error / SENSITIVITY);
      float pid_out = PID_Compute(&pid_center, 0.0f, compressed, dt);

      /* Right motor is inverted in firmware, so swap pid_out signs to correct
       * steering direction. */
      float raw_l = effective_base - pid_out;
      float raw_r = effective_base + pid_out;

      uint8_t speed_l = apply_factor(raw_l, g_left_factor);
      uint8_t speed_r = apply_factor(raw_r, g_right_factor);

      L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, speed_l);
      L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, speed_r);

      printf("[FSM] FWD | L:%.1f R:%.1f Err:%.1f PID:%.2f "
             "PWM_L:%d PWM_R:%d MB:%.1f BASE:%d\r\n",
             g_filt_left, g_filt_right, wall_error, pid_out, speed_l, speed_r,
              margin, (int)effective_base);
    }

    /* Loop rate ~120 ms from three 40 ms sensor delays */
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
void TIM_Init(void) {
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
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64 - 1; /* 64 MHz / 64 = 1 MHz tick */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1; /* 1 kHz PWM, matches PWM_Max=1000 */
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

/* ------------------------------------------------------------------ */
/* USART1: PA9 TX (AF push-pull) / PA10 RX (input pull-up), 115200-8-N-1.
 * Used by the ESP8266 driver. */
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

  _WIFI_USART.Instance = USART1;
  _WIFI_USART.Init.BaudRate = 115200;
  _WIFI_USART.Init.WordLength = UART_WORDLENGTH_8B;
  _WIFI_USART.Init.StopBits = UART_STOPBITS_1;
  _WIFI_USART.Init.Parity = UART_PARITY_NONE;
  _WIFI_USART.Init.Mode = UART_MODE_TX_RX;
  _WIFI_USART.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  _WIFI_USART.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&_WIFI_USART) != HAL_OK)
    while (1)
      ;

  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&_WIFI_USART); }

/* HAL calls this after each single-byte RX IT completes.
 * Wifi_RxCallBack() pushes the byte into the ring and re-arms the next IT. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1)
    Wifi_RxCallBack();
}