#include "wifi_control.h"
#include "esp_ring.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define WIFI_CTRL_SPEED   70
#define SENSOR_REPORT_MS  500
#define WIFI_TCP_PORT     1234

static int8_t   active_link      = -1;
static uint32_t last_report_tick = 0;

/* Helpers */

static void execute_cmd(const char *cmd, L298N_t *motors) {
  if (strncmp(cmd, "FORWARD", 7) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    printf("motor: forward speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "BACK", 4) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    printf("motor: back speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "LEFT", 4) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    printf("motor: left speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "RIGHT", 5) == 0) {
    L298N_Move(motors, MOTOR_LEFT,  MOTOR_FORWARD,  WIFI_CTRL_SPEED);
    L298N_Move(motors, MOTOR_RIGHT, MOTOR_BACKWARD, WIFI_CTRL_SPEED);
    printf("motor: right speed=%d\n", WIFI_CTRL_SPEED);
  } else if (strncmp(cmd, "STOP", 4) == 0) {
    L298N_StopAll(motors);
    printf("motor: stop\n");
  } else {
    printf("motor: unknown cmd: %s\n", cmd);
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
  printf("wifi: sensors: %s", report);
}

/* Parse one async event from the start of buf[0..len).
 * Returns bytes consumed (> 0), or 0 if the event is incomplete. */
static uint16_t process_one_wifi_event(const char *buf, uint16_t len,
                                       L298N_t *motors) {
  if (len == 0) return 0;

  /* +IPD,<id>,<n>:payload */
  if (len >= 5 && strncmp(buf, "+IPD,", 5) == 0) {
    const char *colon = strchr(buf, ':');
    if (!colon) return 0; /* header still arriving */
    int id = 0, n = 0;
    if (sscanf(buf, "+IPD,%d,%d:", &id, &n) == 2 && n > 0) {
      uint16_t hdr = (uint16_t)(colon - buf) + 1;
      if (hdr + (uint16_t)n > len) return 0; /* payload still arriving */
      char payload[64];
      uint16_t cp = ((uint16_t)n < sizeof(payload) - 1)
                        ? (uint16_t)n : (uint16_t)(sizeof(payload) - 1);
      memcpy(payload, buf + hdr, cp);
      payload[cp] = '\0';
      while (cp > 0 && (payload[cp - 1] == '\r' || payload[cp - 1] == '\n'))
        payload[--cp] = '\0';
      active_link = (int8_t)id;
      if (cp > 0) {
        execute_cmd(payload, motors);
        printf("wifi: cmd from link %d: %s\n", id, payload);
      }
      return hdr + (uint16_t)n;
    }
    return 5; /* malformed +IPD - skip past it */
  }

  /* <id>,CONNECT\r\n  /  <id>,CLOSED\r\n */
  if (buf[0] >= '0' && buf[0] <= '4' && len >= 2 && buf[1] == ',') {
    if (len >= 9 && strncmp(buf + 2, "CONNECT", 7) == 0) {
      const char *eol = strchr(buf, '\n');
      if (!eol) return 0;
      active_link = (int8_t)(buf[0] - '0');
      printf("wifi: client %d connected.\n", active_link);
      return (uint16_t)(eol - buf) + 1;
    }
    if (len >= 8 && strncmp(buf + 2, "CLOSED", 6) == 0) {
      const char *eol = strchr(buf, '\n');
      if (!eol) return 0;
      int8_t cid = (int8_t)(buf[0] - '0');
      if (active_link == cid) {
        active_link = -1;
        L298N_StopAll(motors);
        printf("wifi: client disconnected. motors stopped.\n");
      }
      return (uint16_t)(eol - buf) + 1;
    }
  }

  return 1; /* unrecognised byte - skip it */
}

static void poll_async_events(L298N_t *motors) {
  char snap[ESP_RING_SIZE];
  uint16_t len = esp_ring_urc_peek((uint8_t *)snap, sizeof(snap) - 1);
  if (len == 0) return;
  snap[len] = '\0';

  uint16_t pos = 0;
  while (pos < len) {
    uint16_t eaten = process_one_wifi_event(snap + pos, len - pos, motors);
    if (eaten == 0) break;
    pos += eaten;
  }
  if (pos > 0) esp_ring_urc_consume(pos);

  /* Drop stale bytes if the URC cursor is falling far behind */
  if (esp_ring_urc_available() > (3 * ESP_RING_SIZE) / 4)
    esp_ring_urc_consume(esp_ring_urc_available());
}

/* Public API */

/* Maximum number of WiFi connection attempts before giving up. */
#define WIFI_MAX_RETRIES     3
/* Boot delay: ESP8266 needs time after power-on to finish its boot banner.
 * With low battery / voltage sag this can take longer than usual. */
#define WIFI_BOOT_DELAY_MS   3000

void WiFiControl_Init(const char *ssid, const char *password) {
  esp_ring_init();

  /*
   * IMPORTANT: Stop all motors BEFORE attempting WiFi connection.
   * The ESP8266 draws up to 300 mA peak during WiFi TX.  If motors are
   * also running, total current can exceed what the battery / regulator
   * can deliver (especially on a low-charge battery), causing a voltage
   * sag that resets/browns-out the ESP8266 mid-handshake -> timeout.
   */
  extern L298N_t motor_driver;
  L298N_StopAll(&motor_driver);
  printf("wifi: motors stopped - reserving current for ESP8266.\n");

  /* Give the ESP8266 plenty of boot time. */
  HAL_Delay(WIFI_BOOT_DELAY_MS);

  /* AT handshake with retry */
  bool esp_ok = false;
  for (uint8_t attempt = 1; attempt <= WIFI_MAX_RETRIES; attempt++) {
  printf("wifi: Init attempt %d/%d ...\n", attempt, WIFI_MAX_RETRIES);

    if (Wifi_Init()) {
      esp_ok = true;
      break;
    }
    printf("wifi: AT handshake failed, retrying after reset ...\n");
    /* Hard-reset the module and wait for reboot */
    Wifi_Restart();
    HAL_Delay(WIFI_BOOT_DELAY_MS);
  }
  if (!esp_ok) {
    printf("wifi: ESP8266 not responding after %d attempts. "
           "Check wiring/power!\n", WIFI_MAX_RETRIES);
    return;
  }
  printf("wifi: ESP8266 ready.\n");

  /* Set station mode */
  if (!Wifi_SetMode(WifiMode_Station)) {
    printf("wifi: Failed to set WiFi mode.\n");
    return;
  }

  /* Connect to AP with retry */
  bool connected = false;
  for (uint8_t attempt = 1; attempt <= WIFI_MAX_RETRIES; attempt++) {
    printf("wifi: Connecting to \"%s\" (attempt %d/%d) ...\n",
           ssid, attempt, WIFI_MAX_RETRIES);

    if (Wifi_Station_ConnectToAp((char *)ssid, (char *)password, NULL)) {
      connected = true;
      break;
    }
    printf("wifi: Connection attempt %d failed.\n", attempt);
    HAL_Delay(2000); /* cool-down between retries */
  }
  if (!connected) {
    printf("wifi: WiFi connection FAILED after %d attempts.\n",
           WIFI_MAX_RETRIES);
    printf("wifi: Possible causes:\n");
    printf("wifi:   - Battery too low (ESP8266 needs ~300mA peak)\n");
    printf("wifi:   - SSID/password mismatch\n");
    printf("wifi:   - Phone hotspot not active\n");
    return;
  }
  printf("wifi: WiFi connected!\n");

  /*
   * Lower the ESP8266 RF TX power after connecting.
   * Default is 82 (max).  Reducing it saves significant current and
   * makes brownouts less likely once the connection is established.
   * Value 40 ~ half power - still good range for a car demo.
   */
  Wifi_SetRfPower(40);

  HAL_Delay(2000);
  Wifi_RxClear();

  if (!Wifi_TcpIp_SetMultiConnection(true)) {
    printf("wifi: Failed to enable multi-connection.\n");
    return;
  }

  if (!Wifi_TcpIp_SetEnableTcpServer(WIFI_TCP_PORT)) {
    printf("wifi: Failed to start TCP server.\n");
    return;
  }

  if (Wifi_GetMyIp())
    printf("wifi: Local IP: %s\n", Wifi.MyIP);
  else
    printf("wifi: Could not read local IP.\n");

  Wifi_RxClear();
  printf("wifi: Control server ready on port %d.\n", WIFI_TCP_PORT);
  printf("wifi: Commands: FORWARD | BACK | LEFT | RIGHT | STOP\n");
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
