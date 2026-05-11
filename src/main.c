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
#define CENTER_MARGIN_CM 2.0f /* dead-band around wall_error = 0       */
#define BASE_SPEED 55         /* nominal forward PWM % (was 42)         */
#define EMA_ALPHA 0.50f       /* ultrasonic EMA smoothing (was 0.60)    */
#define SENSITIVITY 20.0f     /* tanh compression knee (cm)             */
#define SPEED_FLOOR 35        /* min PWM % while doing wall correction  */

/* ---- Front-obstacle thresholds --------------------------------------- */
#define FRONT_DECEL_CM 35.0f /* Start slowing down & check for turns   */
#define FRONT_STOP_CM 20.0f  /* Final stop/dead-end threshold          */

/* ---- Turn / navigation ----------------------------------------------- */
#define D_FAR_CM 30.0f        /* side dist > D_FAR → corridor is open   */
#define OPEN_WALL_CM 20.0f    /* side dist above this → wall not present */
#define DESIRED_WALL_CM 15.0f /* single-wall follow target              */
/* ---- Turn / navigation (PD) ------------------------------------------ */
#define TURN_KP 7.0f
#define TURN_KD 0.7f
#define TURN_MAX_SPEED 70
#define TURN_MIN_SPEED 35
#define TURN_STOP_DEG 2.5f
#define TURN_TIMEOUT_MS 6000

/* Approach zone: when remaining drops below this, we slow down hard */
#define TURN_APPROACH_DEG 15.0f /* enter slow zone at 15° remaining  */
#define TURN_APPROACH_MAX 32    /* PWM ceiling inside slow zone (%)   */
#define TURN_APPROACH_MIN 26    /* stiction floor inside slow zone    */

#define TURN_ANGLE_DEG 90.0f /* rotation target magnitude              */

/* ---- Wall-to-heading gain -------------------------------------------- */
#define WALL_TO_HEADING_GAIN_DEFAULT 0.50f

/* ---- Heading-hold PD ------------------------------------------------- */
#define HEADING_KP_DEFAULT 3.0f
#define HEADING_KD_DEFAULT 0.45f

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

/* ---- FSM states ------------------------------------------------------ */
typedef enum {
  STATE_IDLE,       /* stopped, waiting for GO                         */
  STATE_FORWARD,    /* corridor-centering + front obstacle check        */
  STATE_TURN_LEFT,  /* executing +90 tank turn                         */
  STATE_TURN_RIGHT, /* executing -90 tank turn                         */
  STATE_DEAD_END,   /* both sides blocked — back up and retry          */
} RobotState;

/* ---- Runtime state --------------------------------------------------- */
static int8_t wifi_link = -1;
static float g_dist_front = 200.0f;
static float g_filt_left = 100.0f;
static float g_filt_right = 100.0f;
static uint8_t g_mode = 0; /* 0=AUTO  1=MANUAL                 */
static uint8_t g_manual_cmd = 0;
static uint8_t g_base_speed = BASE_SPEED;
static float g_left_factor = LEFT_MOTOR_FACTOR;
static float g_right_factor = RIGHT_MOTOR_FACTOR;

/* Shared state written by main loop, consumed by gyro_tick() */
static uint8_t g_running = 1;
static float g_current_base = BASE_SPEED;
static float g_heading_kp = HEADING_KP_DEFAULT;
static float g_heading_kd = HEADING_KD_DEFAULT;
static float g_wall_gain = WALL_TO_HEADING_GAIN_DEFAULT;
static float g_target_angle = 0.0f;

/* FSM */
static RobotState g_state = STATE_FORWARD;

static void apply_motor(uint8_t motor, int pct) {
  float factor = (motor == MOTOR_LEFT) ? g_left_factor : g_right_factor;
  int v = (int)((float)pct * factor);

  if (v > 100)
    v = 100;
  if (v < -100)
    v = -100;

  if (v == 0) {
    L298N_Stop(&motor_driver, motor);
    return;
  }

  uint8_t dir = (v > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint8_t speed = (uint8_t)(v > 0 ? v : -v);
  L298N_Move(&motor_driver, motor, dir, speed);
}

/* -----------------------------------------------------------------------
 * move_forward – drive both motors forward at the given percentage.
 * --------------------------------------------------------------------- */
static void move_forward(int pct) {
  apply_motor(MOTOR_LEFT, pct);
  apply_motor(MOTOR_RIGHT, pct);
}

/* -----------------------------------------------------------------------
 * move_backward – drive both motors backward at the given percentage.
 * --------------------------------------------------------------------- */
static void move_backward(int pct) {
  apply_motor(MOTOR_LEFT, -pct);
  apply_motor(MOTOR_RIGHT, -pct);
}

/* -----------------------------------------------------------------------
 * stop_motors – halt both motors immediately.
 * --------------------------------------------------------------------- */
static void stop_motors(void) { L298N_StopAll(&motor_driver); }

/* -----------------------------------------------------------------------
 * gyro_tick – heading-hold PD inner loop, called every 5 ms.
 *
 * Only active in STATE_FORWARD — cannot interfere with turns.
 * g_target_angle is set each main-loop iteration from the wall error:
 *   Centered  → target ≈ current angle → drive straight
 *   Off-center → target biased inward  → gentle arc back to center
 * --------------------------------------------------------------------- */
static void gyro_tick(void) {
  if (!(g_running && g_mode == 0 && g_state == STATE_FORWARD))
    return;

  MPU6050_UpdateAngle(&imu);

  float remaining = g_target_angle - imu.angle_z;
  float rate_z = imu.gyro.z - imu.gyro_z_offset;
  float corr = g_heading_kp * remaining - g_heading_kd * rate_z;

  float raw_l = g_current_base - corr;
  float raw_r = g_current_base + corr;

  apply_motor(MOTOR_LEFT, (int)raw_l);
  apply_motor(MOTOR_RIGHT, (int)raw_r);
}

/* -----------------------------------------------------------------------
 * sensor_acquire – fire all 3 ultrasonics sequentially.
 * gyro_tick() runs every 5 ms during each echo wait.
 * --------------------------------------------------------------------- */
#define US_TIMEOUT_MS 40
#define GYRO_PERIOD_MS 5

static void sensor_acquire(float *out_front, float *out_left,
                           float *out_right) {
  HC_SR04_t *sensors[3] = {&ultrasonic_front, &ultrasonic_left,
                           &ultrasonic_right};

  for (int i = 0; i < 3; i++) {
    HC_SR04_StartMeasurement(sensors[i]);
    uint32_t t0 = HAL_GetTick();
    while (!HC_SR04_IsDone(sensors[i])) {
      if ((HAL_GetTick() - t0) >= US_TIMEOUT_MS)
        break;
      gyro_tick();
      HAL_Delay(GYRO_PERIOD_MS);
    }
  }

  *out_front = HC_SR04_GetDistance(&ultrasonic_front);
  *out_left = HC_SR04_GetDistance(&ultrasonic_left);
  *out_right = HC_SR04_GetDistance(&ultrasonic_right);
}

/* -----------------------------------------------------------------------
 * do_turn – blocking IMU-guided tank turn.
 *
 *   delta_deg > 0 → LEFT  (CCW: left motor backward, right forward)
 *   delta_deg < 0 → RIGHT (CW:  left motor forward,  right backward)
 *
 * g_state must be STATE_TURN_LEFT/RIGHT before calling so gyro_tick()
 * does not interfere. After completion g_target_angle is updated so
 * corridor-centering resumes from the new heading with no jump.
 * --------------------------------------------------------------------- */
static void do_turn(float delta_deg) {
  if (fabsf(delta_deg) < 1.0f) {
    printf("[TURN] delta too small, skipped\r\n");
    return;
  }

  MPU6050_UpdateAngle(&imu);
  float target = imu.angle_z + delta_deg;
  printf("[TURN] start=%.1f  target=%.1f  delta=%.1f\r\n", imu.angle_z, target,
         delta_deg);

  uint32_t t_start = HAL_GetTick();
  uint32_t last_print = t_start;

  while (1) {
    MPU6050_UpdateAngle(&imu);

    float remaining = target - imu.angle_z;        /* signed error  */
    float rate_z = imu.gyro.z - imu.gyro_z_offset; /* signed deg/s  */

    /* PD output: Positive → CCW (left backward, right forward) */
    float output = TURN_KP * remaining - TURN_KD * rate_z;

    /* Done when error is small AND the car has stopped rotating */
    if (fabsf(remaining) <= TURN_STOP_DEG && fabsf(rate_z) < 15.0f)
      break;

    if ((HAL_GetTick() - t_start) >= TURN_TIMEOUT_MS) {
      printf("[TURN] TIMEOUT\r\n");
      break;
    }

    /* Approach zone: soften limits when close */
    bool approaching = (fabsf(remaining) < TURN_APPROACH_DEG);
    int max_spd = approaching ? TURN_APPROACH_MAX : TURN_MAX_SPEED;
    int min_spd = approaching ? TURN_APPROACH_MIN : TURN_MIN_SPEED;

    float abs_out = fabsf(output);

    /* Stiction floor — only when output agrees with error direction */
    if (abs_out < min_spd) {
      bool pushing_toward = (output > 0.0f && remaining > 0.0f) ||
                            (output < 0.0f && remaining < 0.0f);
      if (pushing_toward)
        abs_out = min_spd;
    }

    if (abs_out > max_spd)
      abs_out = max_spd;

    int spd = (output >= 0.0f) ? (int)abs_out : -(int)abs_out;

    /* Tank spin: left opposite of right */
    apply_motor(MOTOR_LEFT, -spd);
    apply_motor(MOTOR_RIGHT, spd);

    if (HAL_GetTick() - last_print >= 100) {
      last_print = HAL_GetTick();
      printf("[TURN] angle=%+7.2f rem=%+6.2f out=%.1f rate=%.1f\r\n",
             imu.angle_z, remaining, output, rate_z);
    }
    HAL_Delay(5);
  }

  /* Active hold-in-place brake pulse to kill cable tension snap-back */
  HAL_GPIO_WritePin(motor_driver.motor_left.IN1_PORT,
                    motor_driver.motor_left.IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_driver.motor_left.IN2_PORT,
                    motor_driver.motor_left.IN2_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_driver.motor_right.IN1_PORT,
                    motor_driver.motor_right.IN1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor_driver.motor_right.IN2_PORT,
                    motor_driver.motor_right.IN2_PIN, GPIO_PIN_SET);
  L298N_SetSpeed(&motor_driver, MOTOR_LEFT, 100);
  L298N_SetSpeed(&motor_driver, MOTOR_RIGHT, 100);
  HAL_Delay(60);

  L298N_StopAll(&motor_driver);
  HAL_Delay(60);
  MPU6050_UpdateAngle(&imu);

  g_target_angle = imu.angle_z; /* resume corridor-centering from here */
  float error = imu.angle_z - target;
  printf("[TURN] done final=%+.2f target=%+.2f error=%+.2f\r\n", imu.angle_z,
         target, error);
}

/* -----------------------------------------------------------------------
 * choose_turn – left-preference turn decision.
 *   +1 = turn left   -1 = turn right   0 = dead end
 * --------------------------------------------------------------------- */
static int8_t choose_turn(float dist_left, float dist_right) {
  if (dist_left > D_FAR_CM)
    return 1;
  if (dist_right > D_FAR_CM)
    return -1;
  return 0;
}

/* -----------------------------------------------------------------------
 * poll_wifi
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

        if (strncmp(payload, "WG=", 3) == 0) {
          float f = strtof(payload + 3, NULL);
          if (f >= 0.0f)
            g_wall_gain = f;
          alen = snprintf(ack, sizeof(ack), "OK WG=%.3f\r\n", g_wall_gain);

        } else if (strncmp(payload, "HKP=", 4) == 0) {
          float f = strtof(payload + 4, NULL);
          if (f >= 0.0f)
            g_heading_kp = f;
          alen = snprintf(ack, sizeof(ack), "OK HKP=%.3f\r\n", g_heading_kp);

        } else if (strncmp(payload, "HKD=", 4) == 0) {
          float f = strtof(payload + 4, NULL);
          if (f >= 0.0f)
            g_heading_kd = f;
          alen = snprintf(ack, sizeof(ack), "OK HKD=%.3f\r\n", g_heading_kd);

        } else if (strncmp(payload, "LFAC=", 5) == 0) {
          float f = strtof(payload + 5, NULL);
          if (f > 0.0f)
            g_left_factor = f;
          alen = snprintf(ack, sizeof(ack), "OK LFAC=%.3f\r\n", g_left_factor);

        } else if (strncmp(payload, "RFAC=", 5) == 0) {
          float f = strtof(payload + 5, NULL);
          if (f > 0.0f)
            g_right_factor = f;
          alen = snprintf(ack, sizeof(ack), "OK RFAC=%.3f\r\n", g_right_factor);

        } else if (strncmp(payload, "MB=", 3) == 0) {
          float mb = strtof(payload + 3, NULL);
          if (mb >= 0.0f)
            *margin = mb;
          alen = snprintf(ack, sizeof(ack), "OK MB=%.2f\r\n", *margin);

        } else if (strncmp(payload, "SPEED=", 6) == 0) {
          int spd = (int)strtof(payload + 6, NULL);
          if (spd < 0)
            spd = 0;
          if (spd > 100)
            spd = 100;
          g_base_speed = (uint8_t)spd;
          alen = snprintf(ack, sizeof(ack), "OK SPEED=%d\r\n", g_base_speed);

        } else if (strncmp(payload, "DATA", 4) == 0) {
          alen = snprintf(ack, sizeof(ack), "L=%.1f F=%.1f R=%.1f\r\n",
                          g_filt_left, g_dist_front, g_filt_right);

        } else if (strncmp(payload, "GET", 3) == 0) {
          alen =
              snprintf(ack, sizeof(ack),
                       "WG=%.3f HKP=%.3f HKD=%.3f "
                       "MB=%.2f SPD=%d "
                       "LFAC=%.3f RFAC=%.3f "
                       "ANG=%.1f TGT=%.1f "
                       "RUN=%d STATE=%d "
                       "L=%.1f F=%.1f R=%.1f\r\n",
                       g_wall_gain, g_heading_kp, g_heading_kd, *margin,
                       g_base_speed, g_left_factor, g_right_factor, imu.angle_z,
                       g_target_angle, (int)*running, (int)g_state, g_filt_left,
                       g_dist_front, g_filt_right);

        } else if (strncmp(payload, "STOP", 4) == 0) {
          *running = 0;
          g_manual_cmd = 0;
          g_state = STATE_IDLE;
          alen = snprintf(ack, sizeof(ack), "OK STOPPED\r\n");

        } else if (strncmp(payload, "GO", 2) == 0 ||
                   strncmp(payload, "AUTO", 4) == 0) {
          g_mode = 0;
          *running = 1;
          g_state = STATE_FORWARD;
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

  imu.last_time = HAL_GetTick();
  g_target_angle = imu.angle_z;

  while (1) {
    /* LED heartbeat */
    static uint32_t led_last = 0;
    uint32_t now = HAL_GetTick();
    if ((now - led_last) >= 2000) {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      led_last = now;
    }

    /* Acquire sensors; gyro_tick() fires every 5 ms inside (STATE_FORWARD only)
     */
    float dist_left, dist_right;
    sensor_acquire(&g_dist_front, &dist_left, &dist_right);

    uint8_t ir_safe = 1; /* IR disabled for testing */

    MPU6050_UpdateAngle(&imu);
    poll_wifi(&margin, &running);

    g_filt_left = EMA_ALPHA * dist_left + (1.0f - EMA_ALPHA) * g_filt_left;
    g_filt_right = EMA_ALPHA * dist_right + (1.0f - EMA_ALPHA) * g_filt_right;

    g_running = running;

    /* ================================================================
     * FSM
     * ============================================================== */
    if (!running) {
      L298N_StopAll(&motor_driver);
      g_state = STATE_IDLE;

    } else if (g_mode == 1) {
      /* ---- MANUAL -------------------------------------------- */
      switch (g_manual_cmd) {
      case 1:
        move_forward(g_base_speed);
        break;
      case 2:
        move_backward(g_base_speed);
        break;
      case 3:
        apply_motor(MOTOR_LEFT, -g_base_speed);
        apply_motor(MOTOR_RIGHT, g_base_speed);
        break;
      case 4:
        apply_motor(MOTOR_LEFT, g_base_speed);
        apply_motor(MOTOR_RIGHT, -g_base_speed);
        break;
      default:
        stop_motors();
        break;
      }

    } else {
      /* ---- AUTO FSM ------------------------------------------ */
      switch (g_state) {

      /* -------------------------------------------------------- */
      case STATE_IDLE:
        L298N_StopAll(&motor_driver);
        break;

      /* -------------------------------------------------------- */
      case STATE_FORWARD: {
        /*
         * Wall-error: single-wall aware.
         *   Both walls present → center between them.
         *   One wall missing  → maintain DESIRED_WALL_CM from the
         *                       visible wall.
         *   Both missing      → go straight (error = 0).
         */
        float wall_error;
        uint8_t left_open = (g_filt_left > OPEN_WALL_CM);
        uint8_t right_open = (g_filt_right > OPEN_WALL_CM);

        if (!left_open && !right_open) {
          wall_error = g_filt_right - g_filt_left;
        } else if (!right_open) {
          wall_error = DESIRED_WALL_CM - g_filt_right;
        } else if (!left_open) {
          wall_error = g_filt_left - DESIRED_WALL_CM;
        } else {
          wall_error = 0.0f;
        }

        if (wall_error > -margin && wall_error < margin)
          wall_error = 0.0f;

        /* Speed: reduce proportionally to wall error AND front distance */
        float abs_err = fabsf(wall_error);
        float t_wall = abs_err / SENSITIVITY;
        if (t_wall > 1.0f)
          t_wall = 1.0f;

        /* Frontal deceleration ramp */
        float t_front = 0.0f;
        if (g_dist_front < FRONT_DECEL_CM) {
          t_front = (FRONT_DECEL_CM - g_dist_front) /
                    (FRONT_DECEL_CM - FRONT_STOP_CM);
          if (t_front > 1.0f)
            t_front = 1.0f;
        }

        float t_combined = (t_wall > t_front) ? t_wall : t_front;
        float effective_base = (float)g_base_speed * (1.0f - t_combined) +
                               (float)SPEED_FLOOR * t_combined;

        /* Heading target: tanh-compressed wall error → degrees */
        float compressed = SENSITIVITY * tanhf(wall_error / SENSITIVITY);
        g_target_angle = imu.angle_z + compressed * g_wall_gain;
        g_current_base = effective_base;

        /* gyro_tick() applies motors in the next sensor_acquire */
        printf("[FSM] FWD | L:%.1f R:%.1f F:%.1f "
               "Err:%.1f TGT:%.1f ANG:%.1f BASE:%d\r\n",
               g_filt_left, g_filt_right, g_dist_front, wall_error,
               g_target_angle, imu.angle_z, (int)effective_base);

        /*
         * Turn evaluation:
         * 1. If in DECEL zone and we see a side opening, take it.
         * 2. If we hit STOP zone, stop and evaluate (turn or dead-end).
         */
        int8_t potential_dir = choose_turn(g_filt_left, g_filt_right);
        bool should_turn_early =
            (g_dist_front < FRONT_DECEL_CM && potential_dir != 0);
        bool must_stop_now = (g_dist_front < FRONT_STOP_CM || !ir_safe);

        if (should_turn_early || must_stop_now) {
          L298N_StopAll(&motor_driver);
          HAL_Delay(80); /* chassis settle */

          printf("[FSM] STOPPED | F:%.1f L:%.1f R:%.1f (Early:%d)\r\n",
                 g_dist_front, g_filt_left, g_filt_right, should_turn_early);

          int8_t dir = choose_turn(g_filt_left, g_filt_right);
          if (dir > 0) {
            printf("[FSM] → TURN_LEFT  (L=%.1f)\r\n", g_filt_left);
            g_state = STATE_TURN_LEFT;
          } else if (dir < 0) {
            printf("[FSM] → TURN_RIGHT (R=%.1f)\r\n", g_filt_right);
            g_state = STATE_TURN_RIGHT;
          } else {
            printf("[FSM] → DEAD_END\r\n");
            g_state = STATE_DEAD_END;
          }
        }
        break;
      }

      /* -------------------------------------------------------- */
      case STATE_TURN_LEFT:
        do_turn(+TURN_ANGLE_DEG);
        { /* flush EMA with first real post-turn reading */
          float ff, fl, fr;
          sensor_acquire(&ff, &fl, &fr);
          g_dist_front = ff;
          g_filt_left = fl;
          g_filt_right = fr;
        }
        g_state = STATE_FORWARD;
        printf("[FSM] → FORWARD after left turn\r\n");
        break;

      /* -------------------------------------------------------- */
      case STATE_TURN_RIGHT:
        do_turn(-TURN_ANGLE_DEG);
        {
          float ff, fl, fr;
          sensor_acquire(&ff, &fl, &fr);
          g_dist_front = ff;
          g_filt_left = fl;
          g_filt_right = fr;
        }
        g_state = STATE_FORWARD;
        printf("[FSM] → FORWARD after right turn\r\n");
        break;

      /* -------------------------------------------------------- */
      case STATE_DEAD_END: {
        printf("[FSM] DEAD_END — backing up\r\n");

        move_backward((int)((float)g_base_speed * 0.7f));
        HAL_Delay(400);
        stop_motors();
        HAL_Delay(100);

        float ff, fl, fr;
        sensor_acquire(&ff, &fl, &fr);
        g_dist_front = ff;
        g_filt_left = EMA_ALPHA * fl + (1.0f - EMA_ALPHA) * g_filt_left;
        g_filt_right = EMA_ALPHA * fr + (1.0f - EMA_ALPHA) * g_filt_right;

        int8_t dir = choose_turn(g_filt_left, g_filt_right);
        if (dir > 0) {
          printf("[FSM] → TURN_LEFT after backup\r\n");
          g_state = STATE_TURN_LEFT;
        } else if (dir < 0) {
          printf("[FSM] → TURN_RIGHT after backup\r\n");
          g_state = STATE_TURN_RIGHT;
        } else {
          printf("[FSM] → IDLE (true dead end)\r\n");
          L298N_StopAll(&motor_driver);
          running = 0;
          g_running = 0;
          g_state = STATE_IDLE;
        }
        break;
      }

      default:
        g_state = STATE_IDLE;
        break;
      }
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