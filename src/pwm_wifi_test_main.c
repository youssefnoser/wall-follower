#include "ESP8266.h"
#include "dwt_stm32_delay.h"
#include "esp_ring.h"
#include "flash_logger.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "l298n.h"
#include "mpu6050.h"
#include "stm32f1xx_hal.h"
#include "wifi_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- Corridor-centering tuning --------------------------------------- */
#define CENTER_MARGIN_CM 2.0f   /* dead-band around wall_error = 0      */
#define BASE_SPEED 42           /* nominal forward PWM %                */
#define EMA_ALPHA 0.45f         /* ultrasonic EMA smoothing             */
#define FRONT_OBSTACLE_CM 28.0f /* emergency stop threshold             */
#define SENSITIVITY 20.0f       /* tanh compression knee (cm)           */
#define SPEED_FLOOR 35          /* minimum PWM % while correcting       */

/* ---- Wall-to-heading gain -------------------------------------------- *
 * Converts lateral wall error (cm) into a heading target bias (degrees). *
 * g_target_angle = imu.angle_z + wall_error * WALL_TO_HEADING_GAIN       *
 *                                                                         *
 * Tune via WiFi: WG=<value>                                               *
 * Start at 0.2, raise toward 0.6. Too high → weaving. Too low → sluggish *
 * re-centering.                                                           */
#define WALL_TO_HEADING_GAIN_DEFAULT 0.4f

/* ---- Heading-hold PD ------------------------------------------------- *
 * gyro_tick() runs every 5 ms during ultrasonic waits.                   *
 *   remaining = g_target_angle − imu.angle_z  (heading error, degrees)  *
 *   rate_z    = imu.gyro.z − offset            (heading rate,  deg/s)   *
 *   corr      = HKP*remaining − HKD*rate_z                              *
 *   raw_l     = base − corr                                              *
 *   raw_r     = base + corr                                              *
 *                                                                         *
 * Tune via WiFi: HKP= / HKD=                                             *
 * HKP: raise until drift corrects within one loop; back off if weaving.  *
 * HKD: raise if heading overshoots when correcting.                      */
#define HEADING_KP_DEFAULT 2.0f
#define HEADING_KD_DEFAULT 0.3f

/* ---- Per-side motor scale factors ------------------------------------ */
#define LEFT_MOTOR_FACTOR 1.0f
#define RIGHT_MOTOR_FACTOR 1.0f

/* ---- WiFi ------------------------------------------------------------ */
#define WIFI_SSID "Youssef's A51"
#define WIFI_PASSWORD "lol123a@@@"

/* ---- Hardware handles ------------------------------------------------ */
UART_HandleTypeDef huart1;
L298N_t motor_driver;
MPU6050_t imu;
HC_SR04_t ultrasonic_front;
HC_SR04_t ultrasonic_left;
HC_SR04_t ultrasonic_right;
IR_Sensor_t ir_sensor_front;

void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void ADC_Init(void);
void USART1_UART_Init(void);
void I2C_Init(void);

/* ---- Runtime state --------------------------------------------------- */
static int8_t wifi_link = -1;
static float g_dist_front = 200.0f;
static float g_filt_left = 100.0f;
static float g_filt_right = 100.0f;
static uint8_t g_mode = 0;       /* 0=AUTO  1=MANUAL                    */
static uint8_t g_manual_cmd = 0; /* 0=stop 1=fwd 2=bwd 3=left 4=right  */
static uint8_t g_base_speed = BASE_SPEED;
static float g_left_factor = LEFT_MOTOR_FACTOR;
static float g_right_factor = RIGHT_MOTOR_FACTOR;

/* Shared state written by main loop, consumed by gyro_tick() */
static uint8_t g_running = 1;
static float g_current_base = BASE_SPEED;

/* Heading-hold state – runtime-tunable via HKP= / HKD= / WG= */
static float g_heading_kp = HEADING_KP_DEFAULT;
static float g_heading_kd = HEADING_KD_DEFAULT;
static float g_wall_gain = WALL_TO_HEADING_GAIN_DEFAULT;
static float g_target_angle = 0.0f;
static uint8_t g_front_obstacle = 0;

/* -----------------------------------------------------------------------
 * apply_factor – scale a raw PWM duty by a per-motor factor and clamp.
 * --------------------------------------------------------------------- */
static inline uint8_t apply_factor(float raw, float factor) {
  int v = (int)(raw * factor);
  if (v > 100)
    v = 100;
  if (v < 0)
    v = 0;
  return (uint8_t)v;
}

/* -----------------------------------------------------------------------
 * gyro_tick – heading-hold PD inner loop, called every 5 ms.
 *
 * g_target_angle is set by the main loop after each ultrasonic cycle:
 *
 *   Centered  (wall_error ≈ 0):  target ≈ current angle  → drive straight
 *   Off-center (wall_error > 0):  target biased inward    → gentle curve back
 *
 * The robot never stops to correct; it arcs back to center while moving
 * forward. No wall PID output is mixed in here — gyro_tick owns actuation.
 * --------------------------------------------------------------------- */
static void gyro_tick(void) {
  if (!(g_running && g_mode == 0))
    return;
  if (g_front_obstacle) {
    L298N_StopAll(&motor_driver);
    return;
  }

  MPU6050_UpdateAngle(&imu);

  float remaining = g_target_angle - imu.angle_z;
  float rate_z = imu.gyro.z - imu.gyro_z_offset;
  float corr = g_heading_kp * remaining - g_heading_kd * rate_z;

  float raw_l = g_current_base - corr;
  float raw_r = g_current_base + corr;

  L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD,
             apply_factor(raw_l, g_left_factor));
  L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD,
             apply_factor(raw_r, g_right_factor));
}

/* -----------------------------------------------------------------------
 * sensor_acquire – fire all 3 ultrasonics sequentially.
 *
 * gyro_tick() runs every 5 ms while waiting for each echo, so heading
 * correction continues at 200 Hz throughout the ~120 ms acquisition
 * window. Front obstacle is checked immediately after its echo so the
 * robot stops within one gyro_tick cycle, not 80 ms later.
 * --------------------------------------------------------------------- */
#define US_TIMEOUT_MS 40
#define GYRO_PERIOD_MS 5

static void sensor_acquire(float *out_front, float *out_left,
                           float *out_right) {
  HC_SR04_t *sensors[3] = {&ultrasonic_front, &ultrasonic_left,
                           &ultrasonic_right};
  g_front_obstacle = 0;

  for (int i = 0; i < 3; i++) {
    HC_SR04_StartMeasurement(sensors[i]);
    uint32_t t0 = HAL_GetTick();

    while (!HC_SR04_IsDone(sensors[i])) {
      if ((HAL_GetTick() - t0) >= US_TIMEOUT_MS)
        break;
      gyro_tick();
      HAL_Delay(GYRO_PERIOD_MS);
    }

    /* Early obstacle check on front sensor only */
    if (i == 0) {
      float front_now = HC_SR04_GetDistance(&ultrasonic_front);
      if (front_now < FRONT_OBSTACLE_CM) {
        g_front_obstacle = 1;
        L298N_StopAll(&motor_driver);
        printf("[SENSOR] obstacle %.1f cm – stopped early\r\n", front_now);
      }
    }
  }

  *out_front = HC_SR04_GetDistance(&ultrasonic_front);
  *out_left = HC_SR04_GetDistance(&ultrasonic_left);
  *out_right = HC_SR04_GetDistance(&ultrasonic_right);
}

/* -----------------------------------------------------------------------
 * poll_wifi – parse +IPD frames from the ESP8266 ring buffer.
 *
 * Supported commands:
 *   GET           – dump all tunable parameters + live sensor readings
 *   DATA          – live sensor readings only
 *   WG=<f>        – wall-to-heading gain
 *   HKP=<f>       – heading-hold proportional gain
 *   HKD=<f>       – heading-hold derivative gain
 *   LFAC=<f>      – left motor scale factor
 *   RFAC=<f>      – right motor scale factor
 *   MB=<f>        – wall-error dead-band (cm)
 *   SPEED=<0-100> – base forward speed
 *   STOP          – halt and switch to manual
 *   GO            – resume AUTO
 *   AUTO          – switch to AUTO mode
 *   FWD/BWD/LEFT/RIGHT – manual drive
 * --------------------------------------------------------------------- */
static void poll_wifi(float *margin, uint8_t *running) {
  char snap[128];
  uint16_t len = esp_ring_urc_peek((uint8_t *)snap, sizeof(snap) - 1);
  if (len == 0)
    return;
  snap[len] = '\0';

  uint16_t pos = 0;
  while (pos < len) {
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

        char ack[192];
        int alen = 0;

        /* ---- WG= wall-to-heading gain ---- */
        if (strncmp(payload, "WG=", 3) == 0) {
          float f = strtof(payload + 3, NULL);
          if (f >= 0.0f)
            g_wall_gain = f;
          alen = snprintf(ack, sizeof(ack), "OK WG=%.3f\r\n", g_wall_gain);

          /* ---- HKP= heading proportional gain ---- */
        } else if (strncmp(payload, "HKP=", 4) == 0) {
          float f = strtof(payload + 4, NULL);
          if (f >= 0.0f)
            g_heading_kp = f;
          alen = snprintf(ack, sizeof(ack), "OK HKP=%.3f\r\n", g_heading_kp);

          /* ---- HKD= heading derivative gain ---- */
        } else if (strncmp(payload, "HKD=", 4) == 0) {
          float f = strtof(payload + 4, NULL);
          if (f >= 0.0f)
            g_heading_kd = f;
          alen = snprintf(ack, sizeof(ack), "OK HKD=%.3f\r\n", g_heading_kd);

          /* ---- LFAC= left motor scale ---- */
        } else if (strncmp(payload, "LFAC=", 5) == 0) {
          float f = strtof(payload + 5, NULL);
          if (f > 0.0f)
            g_left_factor = f;
          alen = snprintf(ack, sizeof(ack), "OK LFAC=%.3f\r\n", g_left_factor);

          /* ---- RFAC= right motor scale ---- */
        } else if (strncmp(payload, "RFAC=", 5) == 0) {
          float f = strtof(payload + 5, NULL);
          if (f > 0.0f)
            g_right_factor = f;
          alen = snprintf(ack, sizeof(ack), "OK RFAC=%.3f\r\n", g_right_factor);

          /* ---- MB= dead-band margin ---- */
        } else if (strncmp(payload, "MB=", 3) == 0) {
          float mb = strtof(payload + 3, NULL);
          if (mb >= 0.0f)
            *margin = mb;
          alen = snprintf(ack, sizeof(ack), "OK MB=%.2f\r\n", *margin);

          /* ---- SPEED= base forward speed ---- */
        } else if (strncmp(payload, "SPEED=", 6) == 0) {
          int spd = (int)strtof(payload + 6, NULL);
          if (spd < 0)
            spd = 0;
          if (spd > 100)
            spd = 100;
          g_base_speed = (uint8_t)spd;
          alen = snprintf(ack, sizeof(ack), "OK SPEED=%d\r\n", g_base_speed);

          /* ---- DATA – live sensor snapshot ---- */
        } else if (strncmp(payload, "DATA", 4) == 0) {
          alen = snprintf(ack, sizeof(ack), "L=%.1f F=%.1f R=%.1f\r\n",
                          g_filt_left, g_dist_front, g_filt_right);

          /* ---- GET – full parameter + state dump ---- */
        } else if (strncmp(payload, "GET", 3) == 0) {
          alen = snprintf(ack, sizeof(ack),
                          "WG=%.3f HKP=%.3f HKD=%.3f "
                          "MB=%.2f SPD=%d "
                          "LFAC=%.3f RFAC=%.3f "
                          "ANG=%.1f TGT=%.1f "
                          "RUN=%d L=%.1f F=%.1f R=%.1f\r\n",
                          g_wall_gain, g_heading_kp, g_heading_kd, *margin,
                          g_base_speed, g_left_factor, g_right_factor,
                          imu.angle_z, g_target_angle, (int)*running,
                          g_filt_left, g_dist_front, g_filt_right);

          /* ---- STOP ---- */
        } else if (strncmp(payload, "STOP", 4) == 0) {
          *running = 0;
          g_manual_cmd = 0;
          alen = snprintf(ack, sizeof(ack), "OK STOPPED\r\n");

          /* ---- GO / AUTO ---- */
        } else if (strncmp(payload, "GO", 2) == 0 ||
                   strncmp(payload, "AUTO", 4) == 0) {
          g_mode = 0;
          *running = 1;
          /* Target is reset from wall error on the next main-loop
           * iteration; no stale heading is latched here.        */
          alen = snprintf(ack, sizeof(ack), "OK AUTO\r\n");

          /* ---- Manual commands ---- */
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

    /* Skip connection-status lines like "0,CLOSED\n" */
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

/* -----------------------------------------------------------------------
 * main
 * --------------------------------------------------------------------- */
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

  I2C_Init();
  {
    uint8_t who = 0;
    HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(imu.hi2c, MPU6050_ADDR << 1, 0x75,
                                            I2C_MEMADD_SIZE_8BIT, &who, 1, 200);
    printf("[IMU] WHO_AM_I=0x%02X  I2C=%s\r\n", who,
           (rc == HAL_OK) ? "OK" : "FAIL");
  }
  MPU6050_Init(&imu);
  MPU6050_SetDLPF(&imu, MPU6050_DLPF_44HZ);
  printf("[IMU] Calibrating...\r\n");
  MPU6050_Calibrate(&imu, 500);
  printf("[IMU] offset=%.4f\r\n", imu.gyro_z_offset);

  float margin = CENTER_MARGIN_CM;
  uint8_t running = 1;
  uint8_t was_stopped = 1;

  imu.last_time = HAL_GetTick();

  while (1) {
    /* LED heartbeat */
    static uint32_t led_last = 0;
    uint32_t now = HAL_GetTick();
    if ((now - led_last) >= 2000) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      led_last = now;
    }

    /* --- Acquire all three sensors; gyro_tick() runs inside --- */
    float dist_left, dist_right;
    sensor_acquire(&g_dist_front, &dist_left, &dist_right);

    uint8_t ir_safe = 1; /* IR disabled for testing */

    MPU6050_UpdateAngle(&imu);
    poll_wifi(&margin, &running);

    g_filt_left = EMA_ALPHA * dist_left + (1.0f - EMA_ALPHA) * g_filt_left;
    g_filt_right = EMA_ALPHA * dist_right + (1.0f - EMA_ALPHA) * g_filt_right;

    g_running = running;

    /* ----------------------------------------------------------------
     * FSM
     * -------------------------------------------------------------- */
    if (!running) {
      L298N_StopAll(&motor_driver);
      was_stopped = 1;

    } else if (g_mode == 1) {
      /* ---- Manual ----------------------------------------------- */
      uint8_t ml = apply_factor((float)g_base_speed, g_left_factor);
      uint8_t mr = apply_factor((float)g_base_speed, g_right_factor);
      switch (g_manual_cmd) {
      case 1:
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, ml);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, mr);
        break;
      case 2:
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_BACKWARD, ml);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_BACKWARD, mr);
        break;
      case 3:
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, 0);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, mr);
        break;
      case 4:
        L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, ml);
        L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, 0);
        break;
      default:
        L298N_StopAll(&motor_driver);
        break;
      }

    } else if (g_front_obstacle || g_dist_front < FRONT_OBSTACLE_CM ||
               !ir_safe) {
      /* ---- Obstacle stop ---------------------------------------- */
      L298N_StopAll(&motor_driver);
      was_stopped = 1;
      printf("[FSM] STOP | F:%.1f L:%.1f R:%.1f\r\n", g_dist_front, g_filt_left,
             g_filt_right);

    } else {
      /* ---- AUTO corridor-centering ------------------------------ *
       *                                                              *
       * Wall error is encoded directly as a heading target bias.    *
       * gyro_tick() runs every 5 ms during the next sensor_acquire  *
       * and continuously arcs the robot back toward center at 200Hz  *
       * without stopping for a discrete correction maneuver.        *
       *                                                              *
       * wall_error > 0  → robot too far right → target biased left  *
       * wall_error < 0  → robot too far left  → target biased right *
       * wall_error = 0  → target = current angle → drive straight   *
       *                                                              *
       * No wall PID. No motor writes here. gyro_tick() owns all     *
       * actuation in AUTO mode.                                     */
      was_stopped = 0;

      float wall_error = g_filt_right - g_filt_left;
      if (wall_error > -margin && wall_error < margin)
        wall_error = 0.0f;

      /* Speed scaling: slow down proportionally to wall error
       * magnitude so large corrections don't happen at full speed. */
      float abs_err = fabsf(wall_error);
      float t = abs_err / SENSITIVITY;
      if (t > 1.0f)
        t = 1.0f;
      float effective_base =
          (float)g_base_speed * (1.0f - t) + (float)SPEED_FLOOR * t;

      /* Heading target: bias inward proportional to lateral error.
       * wall_error is passed through tanh to prevent large errors
       * from commanding extreme heading angles.                    */
      float compressed = SENSITIVITY * tanhf(wall_error / SENSITIVITY);
      g_target_angle = imu.angle_z + compressed * g_wall_gain;
      g_current_base = effective_base;

      /* gyro_tick() will apply motors within 5 ms — no L298N call
       * needed here. Log the intent for telemetry.                */
      printf("[FSM] FWD | L:%.1f R:%.1f Err:%.1f "
             "TGT:%.1f ANG:%.1f BASE:%d\r\n",
             g_filt_left, g_filt_right, wall_error, g_target_angle, imu.angle_z,
             (int)effective_base);
    }
  }
  return 0;
}

/* ---- IRQ handlers ---------------------------------------------------- */

void SysTick_Handler(void) { HAL_IncTick(); }

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) { (void)GPIO_Pin; }

void TIM3_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_front.htim); }
void TIM4_IRQHandler(void) { HAL_TIM_IRQHandler(ultrasonic_left.htim); }

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&_WIFI_USART); }
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1)
    Wifi_RxCallBack();
}

/* ---- Peripheral init ------------------------------------------------- */

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
  motor_driver.motor_left.PWM_Channel = TIM_CHANNEL_1;
  motor_driver.motor_right.PWM_Channel = TIM_CHANNEL_2;

  ultrasonic_front.TRIG_PORT = GPIOB;
  ultrasonic_front.TRIG_PIN = GPIO_PIN_8;
  ultrasonic_front.ECHO_PORT = GPIOA;
  ultrasonic_front.ECHO_PIN = GPIO_PIN_6;
  ultrasonic_left.TRIG_PORT = GPIOA;
  ultrasonic_left.TRIG_PIN = GPIO_PIN_7;
  ultrasonic_left.ECHO_PORT = GPIOB;
  ultrasonic_left.ECHO_PIN = GPIO_PIN_9;
  ultrasonic_right.TRIG_PORT = GPIOB;
  ultrasonic_right.TRIG_PIN = GPIO_PIN_10;
  ultrasonic_right.ECHO_PORT = GPIOB;
  ultrasonic_right.ECHO_PIN = GPIO_PIN_0;

  ir_sensor_front.GPIO_PORT = GPIOA;
  ir_sensor_front.GPIO_PIN = GPIO_PIN_4;
  ir_sensor_front.mode = IR_MODE_DIGITAL;

  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_13;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &g);
}

void TIM_Init(void) {
  static TIM_HandleTypeDef htim2, htim3, htim4;
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* TIM2 – PWM for motors */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 63;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  motor_driver.motor_left.PWM_Max = 1000;
  motor_driver.motor_right.htim = &htim2;
  motor_driver.motor_right.PWM_Max = 1000;

  /* TIM3 – IC for front + right ultrasonics */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 63;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xFFFF;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_IC_Init(&htim3);

  /* TIM4 – IC for left ultrasonic */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
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
  HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3);
  ultrasonic_right.htim = &htim3;
  ultrasonic_right.tim_channel = TIM_CHANNEL_3;
  HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4);
  ultrasonic_left.htim = &htim4;
  ultrasonic_left.tim_channel = TIM_CHANNEL_4;

  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);

  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void ADC_Init(void) {}

void I2C_Init(void) {
  static I2C_HandleTypeDef hi2c1;
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_AF_OD;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &g);
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
  GPIO_InitTypeDef g = {0};
  g.Pin = GPIO_PIN_9;
  g.Mode = GPIO_MODE_AF_PP;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_10;
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);
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