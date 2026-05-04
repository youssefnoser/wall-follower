#include "wifi_control.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define WIFI_CTRL_SPEED   70
#define SENSOR_REPORT_MS  500

static int8_t  active_link      = -1;
static uint32_t last_report_tick = 0;

/* -----------------------------------------------------------------------
 * Private helpers
 * --------------------------------------------------------------------- */

static void execute_cmd(const char *cmd, L298N_t *motors) {
  if (strncmp(cmd, "FORWARD", 7) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_FORWARD,  WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "BACK", 4) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_BACKWARD, WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "LEFT", 4) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_FORWARD,  WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "RIGHT", 5) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_BACKWARD, WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "STOP", 4) == 0) {
    L298N_StopAll(motors);
  }
}

/* Trigger all 3 ultrasonics in sequence, read IMU, then push the report. */
static void send_sensor_report(HC_SR04_t *front, HC_SR04_t *left,
                               HC_SR04_t *right, MPU6050_t *imu) {
  HC_SR04_StartMeasurement(front);
  HAL_Delay(50);
  HC_SR04_StartMeasurement(left);
  HAL_Delay(50);
  HC_SR04_StartMeasurement(right);
  HAL_Delay(50);
  MPU6050_ReadAll(imu);

  float df = HC_SR04_GetDistance(front);
  float dl = HC_SR04_GetDistance(left);
  float dr = HC_SR04_GetDistance(right);

  char report[128];
  snprintf(report, sizeof(report),
           "F=%.1f,L=%.1f,R=%.1f,AX=%.2f,AY=%.2f,AZ=%.2f,GX=%.2f,GY=%.2f,GZ=%.2f\r\n",
           df, dl, dr,
           imu->accel.x, imu->accel.y, imu->accel.z,
           imu->gyro.x,  imu->gyro.y,  imu->gyro.z);

  ESP8266_SendToClient((uint8_t)active_link, report);
  printf("[WiFi] sensors: %s", report);
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void WiFiControl_Init(const char *ssid, const char *password) {
  ESP8266_Init();
  HAL_Delay(1000); /* Give ESP8266 time to boot after power-on */

  /* Disable echo so responses are clean (no command text echoed back) */
  ESP8266_DisableEcho();

  /* Verify module is alive */
  if (!ESP8266_Test()) {
    printf("[WiFi] ESP8266 not responding!\n");
    return;
  }
  printf("[WiFi] ESP8266 ready.\n");

  /* Set Station + SoftAP mode so we can join a network AND run a server */
  if (!ESP8266_SetMode(1)) {
    printf("[WiFi] Failed to set WiFi mode.\n");
    return;
  }

  /* Connect to the access point */
  printf("[WiFi] Connecting to %s ...\n", ssid);
  if (!ESP8266_ConnectWiFi(ssid, password)) {
    printf("[WiFi] WiFi connection failed.\n");
    return;
  }
  printf("[WiFi] WiFi connected.\n");

  /* Enable multiple connections (required for TCP server) */
  if (!ESP8266_SetMux(1)) {
    printf("[WiFi] Failed to set MUX mode.\n");
    return;
  }

  /* Start TCP server on port 1234 */
  if (!ESP8266_StartServer(1234)) {
    printf("[WiFi] Failed to start server.\n");
    return;
  }

  /* Print local IP for user reference */
  ESP8266_GetIP();

  printf("[WiFi] Control server ready on port 1234.\n");
  printf("[WiFi] Commands: FORWARD | BACK | LEFT | RIGHT | STOP\n");
}

/*
 * Call this every main-loop iteration.
 *
 * Receives:  +IPD,<id>,<len>:<cmd>  →  executes motor command
 *            <id>,CONNECT           →  records active client
 *            <id>,CLOSED            →  stops motors, clears client
 *
 * Sends:     sensor report string every SENSOR_REPORT_MS to active client.
 */
void WiFiControl_Update(L298N_t *motors, HC_SR04_t *front, HC_SR04_t *left,
                        HC_SR04_t *right, MPU6050_t *imu) {
  char rx_buf[64] = {0};

  if (ESP8266_ReadLine(rx_buf, sizeof(rx_buf), 10)) {
    if (strncmp(rx_buf, "+IPD", 4) == 0) {
      /* +IPD,<link_id>,<len>:<payload> */
      uint8_t link_id = 0;
      if (rx_buf[5] >= '0' && rx_buf[5] <= '9') {
        link_id    = (uint8_t)(rx_buf[5] - '0');
        active_link = (int8_t)link_id;
      }
      char *payload = strchr(rx_buf, ':');
      if (payload) {
        payload++;
        uint16_t plen = (uint16_t)strlen(payload);
        while (plen > 0 &&
               (payload[plen - 1] == '\r' || payload[plen - 1] == '\n'))
          payload[--plen] = '\0';
        if (plen > 0) {
          execute_cmd(payload, motors);
          printf("[WiFi] cmd from link %d: %s\n", link_id, payload);
        }
      }
    } else if (rx_buf[1] == ',' && strncmp(rx_buf + 2, "CONNECT", 7) == 0) {
      active_link = (int8_t)(rx_buf[0] - '0');
      printf("[WiFi] client %d connected.\n", active_link);
    } else if (rx_buf[1] == ',' && strncmp(rx_buf + 2, "CLOSED", 6) == 0) {
      if (active_link == (int8_t)(rx_buf[0] - '0')) {
        active_link = -1;
        L298N_StopAll(motors);
        printf("[WiFi] client disconnected. Motors stopped.\n");
      }
    }
  }

  uint32_t now = HAL_GetTick();
  if (active_link >= 0 && (now - last_report_tick) >= SENSOR_REPORT_MS) {
    last_report_tick = now;
    send_sensor_report(front, left, right, imu);
  }
}
