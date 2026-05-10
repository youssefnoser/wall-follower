/*
 * pwm_wifi_test_main.c
 *
 * Minimal test bench: brings up WiFi + a TCP server on port 1234, and lets
 * a remote client adjust the PWM duty of each motor independently.
 *
 * Protocol (newline-terminated ASCII, case-insensitive):
 *   L <pct>   set LEFT  motor, signed -100..100 (sign = direction, 0 = stop)
 *   R <pct>   set RIGHT motor, signed -100..100
 *   B <pct>   set BOTH  motors
 *   STOP      stop all motors
 *   ?         report current PWM values back to client
 *
 * To build with this file instead of src/main.c, edit CMakeLists.txt and
 * replace "src/main.c" with "src/pwm_wifi_test_main.c".
 */

#include "ESP8266.h"
#include "dwt_stm32_delay.h"
#include "esp_ring.h"
#include "l298n.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define WIFI_SSID "Yousef"
#define WIFI_PASSWORD "20202020"
#define WIFI_TCP_PORT 1234
#define WIFI_BOOT_DELAY_MS 3000
#define WIFI_MAX_RETRIES 3

L298N_t motor_driver;
static int8_t active_link = -1;
static int8_t left_pwm_signed = 0; /* cached so '?' can report */
static int8_t right_pwm_signed = 0;

void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void USART1_UART_Init(void);

/* ------------------------------------------------------------------ */
static void apply_motor(uint8_t motor, int pct) {
  if (pct > 100)
    pct = 100;
  if (pct < -100)
    pct = -100;

  if (motor == MOTOR_LEFT)
    left_pwm_signed = (int8_t)pct;
  else
    right_pwm_signed = (int8_t)pct;

  if (pct == 0) {
    L298N_Stop(&motor_driver, motor);
    return;
  }
  uint8_t dir = (pct > 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  uint8_t speed = (uint8_t)(pct > 0 ? pct : -pct);

  /* ── ADD THESE ── */
  L298N_Motor_t *dbg = (motor == MOTOR_LEFT) ? &motor_driver.motor_left
                                             : &motor_driver.motor_right;
  printf("[DBG] motor=%d htim=%p PWM_Max=%lu ch=0x%lX speed=%d\n", motor,
         (void *)dbg->htim, dbg->PWM_Max, dbg->PWM_Channel, speed);
  /* ────────────── */

  L298N_Move(&motor_driver, motor, dir, speed);
}

static void send_ack(const char *msg) {
  if (active_link < 0)
    return;
  uint16_t n = (uint16_t)strlen(msg);
  Wifi_TcpIp_SendDataTcp((uint8_t)active_link, n, (uint8_t *)msg);
}

static void handle_command(char *cmd) {
  /* upper-case in place */
  for (char *p = cmd; *p; ++p)
    if (*p >= 'a' && *p <= 'z')
      *p = (char)(*p - 'a' + 'A');

  if (strcmp(cmd, "STOP") == 0) {
    L298N_StopAll(&motor_driver);
    left_pwm_signed = right_pwm_signed = 0;
    send_ack("OK STOP\r\n");
    printf("[Test] STOP\n");
    return;
  }
  if (strcmp(cmd, "?") == 0) {
    char rep[48];
    int n = snprintf(rep, sizeof(rep), "L=%d R=%d\r\n", left_pwm_signed,
                     right_pwm_signed);
    if (n > 0)
      send_ack(rep);
    return;
  }

  /* L <pct> | R <pct> | B <pct> */
  char which = cmd[0];
  if ((which == 'L' || which == 'R' || which == 'B') &&
      (cmd[1] == ' ' || cmd[1] == '\t')) {
    int pct = atoi(cmd + 2);
    if (which == 'L' || which == 'B')
      apply_motor(MOTOR_LEFT, pct);
    if (which == 'R' || which == 'B')
      apply_motor(MOTOR_RIGHT, pct);

    char rep[48];
    int n = snprintf(rep, sizeof(rep), "OK %c=%d\r\n", which, pct);
    if (n > 0)
      send_ack(rep);
    printf("[Test] %c = %d\n", which, pct);
    return;
  }

  send_ack("ERR\r\n");
  printf("[Test] unknown cmd: %s\n", cmd);
}

/* ------------------------------------------------------------------ *
 * Parse one event starting at buf[0..len-1].  Same pattern as the
 * stability test: locate the earliest recognisable marker via strstr
 * so AT-command leftovers are skipped in one jump instead of byte-by-byte.
 * Returns bytes consumed (>0) when an event was handled, or 0 if the
 * event is incomplete (caller should wait for more bytes).
 * ------------------------------------------------------------------ */
static uint16_t process_one_event(const char *buf, uint16_t len) {
  if (len == 0)
    return 0;

  const char *ipd = strstr(buf, "+IPD,");

  /* "<digit>,CONNECT" / "<digit>,CLOSED" can appear anywhere in the snap. */
  const char *tcp_evt = NULL;
  for (uint16_t i = 0; i + 8 <= len; i++) {
    if (buf[i] >= '0' && buf[i] <= '4' && buf[i + 1] == ',' &&
        (strncmp(buf + i + 2, "CONNECT", 7) == 0 ||
         strncmp(buf + i + 2, "CLOSED", 6) == 0)) {
      tcp_evt = buf + i;
      break;
    }
  }

  const char *earliest = NULL;
  if (ipd)
    earliest = ipd;
  if (tcp_evt && (!earliest || tcp_evt < earliest))
    earliest = tcp_evt;

  if (!earliest)
    return 0; /* nothing recognised — keep bytes, wait */

  /* ---- +IPD,id,n:payload --------------------------------------------- */
  if (earliest == ipd) {
    const char *colon = strchr(ipd, ':');
    if (!colon)
      return 0; /* header still arriving */
    int id = 0, n = 0;
    if (sscanf(ipd, "+IPD,%d,%d:", &id, &n) != 2 || n <= 0) {
      /* Malformed — skip past "+IPD," and retry. */
      return (uint16_t)(ipd - buf) + 5;
    }
    uint16_t header_end = (uint16_t)(colon - buf) + 1;
    uint16_t need = header_end + (uint16_t)n;
    if (need > len)
      return 0; /* payload still arriving */

    char payload[64];
    uint16_t cp = (uint16_t)n < sizeof(payload) - 1
                      ? (uint16_t)n
                      : (uint16_t)(sizeof(payload) - 1);
    memcpy(payload, buf + header_end, cp);
    payload[cp] = '\0';
    while (cp > 0 && (payload[cp - 1] == '\r' || payload[cp - 1] == '\n'))
      payload[--cp] = '\0';
    active_link = (int8_t)id;
    if (cp > 0)
      handle_command(payload);
    return need;
  }

  /* ---- "<digit>,CONNECT" / "<digit>,CLOSED" -------------------------- */
  const char *eol = strchr(tcp_evt, '\n');
  if (!eol)
    return 0;
  if (strncmp(tcp_evt + 2, "CONNECT", 7) == 0) {
    active_link = (int8_t)(tcp_evt[0] - '0');
    printf("[Test] client %d connected.\n", active_link);
  } else {
    int8_t cid = (int8_t)(tcp_evt[0] - '0');
    if (active_link == cid) {
      active_link = -1;
      L298N_StopAll(&motor_driver);
      left_pwm_signed = right_pwm_signed = 0;
      printf("[Test] client disconnected. Motors stopped.\n");
    }
  }
  return (uint16_t)(eol - buf) + 1;
}

static void poll_async_events(void) {
  char snap[ESP_RING_SIZE];
  uint16_t len = esp_ring_urc_peek((uint8_t *)snap, sizeof(snap) - 1);
  if (len == 0)
    return;
  snap[len] = '\0';

  uint16_t pos = 0;
  while (pos < len) {
    uint16_t eaten = process_one_event(snap + pos, len - pos);
    if (eaten == 0)
      break;
    pos += eaten;
  }
  if (pos > 0)
    esp_ring_urc_consume(pos);

  if (esp_ring_urc_available() > (3 * ESP_RING_SIZE) / 4)
    esp_ring_urc_consume(esp_ring_urc_available());
}

/* ------------------------------------------------------------------ */
static void wifi_bringup(void) {
  esp_ring_init();
  L298N_StopAll(&motor_driver);
  HAL_Delay(WIFI_BOOT_DELAY_MS);

  bool ok = false;
  for (uint8_t a = 1; a <= WIFI_MAX_RETRIES; a++) {
    printf("[WiFi] init %d/%d\n", a, WIFI_MAX_RETRIES);
    if (Wifi_Init()) {
      ok = true;
      break;
    }
    Wifi_Restart();
    HAL_Delay(WIFI_BOOT_DELAY_MS);
  }
  if (!ok) {
    printf("[WiFi] ESP not responding\n");
    return;
  }

  if (!Wifi_SetMode(WifiMode_Station)) {
    printf("[WiFi] mode set fail\n");
    return;
  }

  bool conn = false;
  for (uint8_t a = 1; a <= WIFI_MAX_RETRIES; a++) {
    printf("[WiFi] connect to %s (%d/%d)\n", WIFI_SSID, a, WIFI_MAX_RETRIES);
    if (Wifi_Station_ConnectToAp((char *)WIFI_SSID, (char *)WIFI_PASSWORD,
                                 NULL)) {
      conn = true;
      break;
    }
    HAL_Delay(2000);
  }
  if (!conn) {
    printf("[WiFi] AP connect failed\n");
    return;
  }

  Wifi_SetRfPower(40);
  HAL_Delay(2000);
  Wifi_RxClear();

  if (!Wifi_TcpIp_SetMultiConnection(true)) {
    printf("[WiFi] mux fail\n");
    return;
  }
  if (!Wifi_TcpIp_SetEnableTcpServer(WIFI_TCP_PORT)) {
    printf("[WiFi] server fail\n");
    return;
  }

  if (Wifi_GetMyIp())
    printf("[WiFi] IP: %s\n", Wifi.MyIP);
  Wifi_RxClear();
  /* Discard all AT-command history from the URC cursor so poll_async_events
     only sees real events (CONNECT / +IPD) that arrive after bringup. */
  esp_ring_urc_consume(esp_ring_urc_available());

  printf("[Test] PWM tester ready on port %d.\n", WIFI_TCP_PORT);
  printf("[Test] cmds: L <pct> | R <pct> | B <pct> | STOP | ?\n");
}

// extern void initialise_monitor_handles(void);

/* ------------------------------------------------------------------ */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  DWT_Delay_Init();
  // initialise_monitor_handles();
  USART1_UART_Init();

  GPIO_Init();
  TIM_Init();

  L298N_Init(&motor_driver);

  wifi_bringup();

  while (1) {
    poll_async_events();
  }
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

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);
}

/* ------------------------------------------------------------------ */
void TIM_Init(void) {
  static TIM_HandleTypeDef htim2;
  TIM_OC_InitTypeDef sConfigOC = {0};

  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64 - 1; /* 1 MHz tick */
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000 - 1; /* 1 kHz PWM */
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
  motor_driver.motor_left.PWM_Max = 1000;
  motor_driver.motor_right.PWM_Max = 1000;
}

/* ------------------------------------------------------------------ */
void USART1_UART_Init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  _WIFI_USART.Instance = USART2;
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

  HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
}

/* ------------------------------------------------------------------ */
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&_WIFI_USART); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2)
    Wifi_RxCallBack();
}
