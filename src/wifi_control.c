#include "wifi_control.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define WIFI_CTRL_SPEED   70
#define SENSOR_REPORT_MS  500
#define WIFI_TCP_PORT     1234

static int8_t   active_link      = -1;
static uint32_t last_report_tick = 0;

/* -----------------------------------------------------------------------
 * Private helpers
 * --------------------------------------------------------------------- */

static void execute_cmd(const char *cmd, L298N_t *motors) {
  if (strncmp(cmd, "FORWARD", 7) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    printf("[Motor] FORWARD speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "BACK", 4) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    printf("[Motor] BACK speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "LEFT", 4) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    printf("[Motor] LEFT speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "RIGHT", 5) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    printf("[Motor] RIGHT speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "STOP", 4) == 0) {
    L298N_StopAll(motors);
    printf("[Motor] STOP\n");
  } else {
    printf("[Motor] unknown cmd: %s\n", cmd);
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

  float df = HC_SR04_GetDistance(front);
  float dl = HC_SR04_GetDistance(left);
  float dr = HC_SR04_GetDistance(right);

  char report[128];
  int n = snprintf(report, sizeof(report),
                   "F=%.1f,L=%.1f,R=%.1f,AX=%.2f,AY=%.2f,AZ=%.2f,GX=%.2f,GY=%.2f,GZ=%.2f\r\n",
                   df, dl, dr,
                   imu->accel.x, imu->accel.y, imu->accel.z,
                   imu->gyro.x,  imu->gyro.y,  imu->gyro.z);
  if (n < 0) return;
  if (n > (int)sizeof(report)) n = (int)sizeof(report);

  Wifi_TcpIp_SendDataTcp((uint8_t)active_link, (uint16_t)n, (uint8_t *)report);
  printf("[WiFi] sensors: %s", report);
}

/* Scan the driver's RX accumulator for async events:
 *   <id>,CONNECT
 *   <id>,CLOSED
 *   +IPD,<id>,<len>:<payload>
 *
 * The driver fills Wifi.RxBuffer byte-by-byte via HAL_UART_Receive_IT.
 * We only consume *complete* events (terminated by '\n' for CONNECT/CLOSED,
 * or once <len> payload bytes have arrived after ':' for +IPD); anything
 * partial is left in place for the next pass. After an event is handled
 * we shift the remaining bytes to the start of the buffer so we don't
 * lose data still in flight.
 */
static void compact_rx_buffer(uint16_t consumed) {
  __disable_irq();
  if (consumed >= Wifi.RxIndex) {
    Wifi.RxIndex = 0;
    Wifi.RxBuffer[0] = 0;
  } else {
    uint16_t left = (uint16_t)(Wifi.RxIndex - consumed);
    memmove(Wifi.RxBuffer, Wifi.RxBuffer + consumed, left);
    Wifi.RxIndex = left;
    memset(Wifi.RxBuffer + left, 0, _WIFI_RX_SIZE - left);
  }
  __enable_irq();
}

static void poll_async_events(L298N_t *motors) {
  if (Wifi.RxIndex == 0) return;

  /* Try to find the first +IPD or x,CONNECT/CLOSED token. */
  char *buf = (char *)Wifi.RxBuffer;
  uint16_t len = Wifi.RxIndex;
  buf[len < _WIFI_RX_SIZE ? len : _WIFI_RX_SIZE - 1] = '\0';

  /* +IPD,<id>,<n>:payload --------------------------------------------- */
  char *ipd = strstr(buf, "+IPD,");
  if (ipd) {
    char *colon = strchr(ipd, ':');
    if (!colon) return; /* header not complete yet */
    int id = 0, n = 0;
    if (sscanf(ipd, "+IPD,%d,%d:", &id, &n) == 2 && n > 0) {
      uint16_t header_len = (uint16_t)(colon - buf) + 1;
      uint16_t need = header_len + (uint16_t)n;
      if (need > Wifi.RxIndex) return; /* payload still arriving */

      char payload[64];
      uint16_t cp = (uint16_t)n < sizeof(payload) - 1 ? (uint16_t)n
                                                      : sizeof(payload) - 1;
      memcpy(payload, buf + header_len, cp);
      payload[cp] = '\0';
      while (cp > 0 && (payload[cp - 1] == '\r' || payload[cp - 1] == '\n'))
        payload[--cp] = '\0';
      active_link = (int8_t)id;
      if (cp > 0) {
        execute_cmd(payload, motors);
        printf("[WiFi] cmd from link %d: %s\n", id, payload);
      }
      compact_rx_buffer(need);
      return;
    }
    /* header malformed but truncated → wait for more */
    return;
  }

  /* <id>,CONNECT / <id>,CLOSED ---------------------------------------- */
  for (uint16_t i = 0; i + 8 <= len; i++) {
    if (buf[i] >= '0' && buf[i] <= '4' && buf[i + 1] == ',') {
      if (strncmp(buf + i + 2, "CONNECT", 7) == 0) {
        char *eol = strchr(buf + i, '\n');
        if (!eol) return; /* incomplete */
        active_link = (int8_t)(buf[i] - '0');
        printf("[WiFi] client %d connected.\n", active_link);
        compact_rx_buffer((uint16_t)(eol - buf + 1));
        return;
      }
      if (strncmp(buf + i + 2, "CLOSED", 6) == 0) {
        char *eol = strchr(buf + i, '\n');
        if (!eol) return;
        int8_t id = (int8_t)(buf[i] - '0');
        if (active_link == id) {
          active_link = -1;
          L298N_StopAll(motors);
          printf("[WiFi] client disconnected. Motors stopped.\n");
        }
        compact_rx_buffer((uint16_t)(eol - buf + 1));
        return;
      }
    }
  }

  /* Nothing useful — but if buffer is filling with garbage that has no
   * recognised token and no in-flight +IPD header, drop it to avoid
   * eventually overflowing _WIFI_RX_SIZE. */
  if (Wifi.RxIndex > _WIFI_RX_SIZE / 2)
    Wifi_RxClear();
}

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */

void WiFiControl_Init(const char *ssid, const char *password) {
  /* USART1 itself is brought up in main.c. We just hand off to the driver. */
  HAL_Delay(1000); /* let the module finish its boot banner */

  if (!Wifi_Init()) {
    printf("[WiFi] ESP8266 not responding (Wifi_Init failed).\n");
    return;
  }
  printf("[WiFi] ESP8266 ready.\n");

  if (!Wifi_SetMode(WifiMode_Station)) {
    printf("[WiFi] Failed to set WiFi mode.\n");
    return;
  }

  printf("[WiFi] Connecting to %s ...\n", ssid);
  if (!Wifi_Station_ConnectToAp((char *)ssid, (char *)password, NULL)) {
    printf("[WiFi] WiFi connection failed.\n");
    return;
  }
  printf("[WiFi] WiFi connected.\n");

  HAL_Delay(2000);
  Wifi_RxClear();

  if (!Wifi_TcpIp_SetMultiConnection(true)) {
    printf("[WiFi] Failed to enable multi-connection.\n");
    return;
  }

  if (!Wifi_TcpIp_SetEnableTcpServer(WIFI_TCP_PORT)) {
    printf("[WiFi] Failed to start TCP server.\n");
    return;
  }

  if (Wifi_GetMyIp())
    printf("[WiFi] Local IP: %s\n", Wifi.MyIP);
  else
    printf("[WiFi] Could not read local IP.\n");

  Wifi_RxClear();
  printf("[WiFi] Control server ready on port %d.\n", WIFI_TCP_PORT);
  printf("[WiFi] Commands: FORWARD | BACK | LEFT | RIGHT | STOP\n");
}

void WiFiControl_Update(L298N_t *motors, HC_SR04_t *front, HC_SR04_t *left,
                        HC_SR04_t *right, MPU6050_t *imu) {
  (void)imu; /* MPU6050_ReadAll skipped per existing convention */

  poll_async_events(motors);

  uint32_t now = HAL_GetTick();
  if (active_link >= 0 && (now - last_report_tick) >= SENSOR_REPORT_MS) {
    last_report_tick = now;
    send_sensor_report(front, left, right, imu);
  }
}
