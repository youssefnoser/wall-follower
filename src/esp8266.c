#include "esp8266.h"
#include <stdio.h>
#include <string.h>

/* Set to 1 to print all AT traffic via semihosting/printf */
#ifndef ESP8266_DEBUG
#define ESP8266_DEBUG 1
#endif

UART_HandleTypeDef huart1;

/* ==================================================================
 *  Internal: configure USART1 at a given baud rate
 * ================================================================== */
static void esp8266_uart_setup(uint32_t baud) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = baud;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_DeInit(&huart1);
  HAL_UART_Init(&huart1);
}

/* Quick "AT" probe – returns true if we get "OK" back */
static bool esp8266_probe(void) {
  HAL_UART_Transmit(&huart1, (uint8_t *)"AT\r\n", 4, HAL_MAX_DELAY);

  char rx_buf[64];
  uint16_t rx_idx = 0;
  uint32_t start = HAL_GetTick();
  uint8_t ch;
  memset(rx_buf, 0, sizeof(rx_buf));

  while ((HAL_GetTick() - start) < 1000) {
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK) {
      if (rx_idx < sizeof(rx_buf) - 1)
        rx_buf[rx_idx++] = ch;
      if (strstr(rx_buf, "OK") != NULL)
        return true;
    }
  }
#if ESP8266_DEBUG
  printf("[ESP] probe buf(%u): %s\n", rx_idx, rx_buf);
#endif
  return false;
}

/* ==================================================================
 *  Hardware Init – USART1 on PA9 (TX) / PA10 (RX)
 *  Tries common baud rates until the module responds to "AT".
 * ================================================================== */
void ESP8266_Init(void) {
  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // PA9 TX -> ESP RX
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA10 RX -> ESP TX
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Try common ESP8266 baud rates */
  static const uint32_t baud_rates[] = {115200, 9600, 57600, 38400, 19200};
  static const uint8_t num_rates = sizeof(baud_rates) / sizeof(baud_rates[0]);

  for (uint8_t i = 0; i < num_rates; i++) {
#if ESP8266_DEBUG
    printf("[ESP] Trying %lu baud...\n", (unsigned long)baud_rates[i]);
#endif
    esp8266_uart_setup(baud_rates[i]);
    HAL_Delay(100);

    if (esp8266_probe()) {
#if ESP8266_DEBUG
      printf("[ESP] Found module at %lu baud.\n", (unsigned long)baud_rates[i]);
#endif
      return; /* success – UART is now locked to the working rate */
    }
  }

  /* Fall back to 115200 if nothing responded */
#if ESP8266_DEBUG
  printf("[ESP] No response at any baud rate! Defaulting to 115200.\n");
#endif
  esp8266_uart_setup(115200);
}

/* ==================================================================
 *  Low-level: drain any stale bytes sitting in the UART RX buffer
 * ================================================================== */
void ESP8266_FlushRX(void) {
  uint8_t dummy;
  while (HAL_UART_Receive(&huart1, &dummy, 1, 5) == HAL_OK)
    ; /* discard */
}

/* ==================================================================
 *  Low-level: send an AT command string (appends \r\n automatically)
 *  Flushes stale RX data first so the next response parse is clean.
 * ================================================================== */
void ESP8266_SendCmd(const char *cmd) {
  ESP8266_FlushRX();
#if ESP8266_DEBUG
  printf("[ESP TX] %s\n", cmd);
#endif
  HAL_UART_Transmit(&huart1, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}

/* ==================================================================
 *  Low-level: read until `expected_response` appears or timeout
 * ================================================================== */
bool ESP8266_CheckResponse(const char *expected_response, uint32_t timeout) {
  char rx_buf[ESP8266_RX_BUF_SIZE];
  uint16_t rx_idx = 0;
  uint32_t start_time = HAL_GetTick();
  uint8_t ch;

  memset(rx_buf, 0, sizeof(rx_buf));

  while ((HAL_GetTick() - start_time) < timeout) {
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK) {
      if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = ch;
        rx_buf[rx_idx] = '\0'; // Null terminate
      }
      if (strstr(rx_buf, expected_response) != NULL) {
#if ESP8266_DEBUG
        printf("[ESP RX] OK got \"%s\" | buf: %s\n", expected_response, rx_buf);
#endif
        return true;
      }
    }
  }
#if ESP8266_DEBUG
  printf("[ESP RX] TIMEOUT waiting for \"%s\" (%lums) | buf(%u): %s\n",
         expected_response, (unsigned long)timeout, rx_idx, rx_buf);
#endif
  return false;
}

/* ==================================================================
 *  Low-level: wait for either a success or failure token
 *  Returns true if `success` token is seen first, false on `fail`
 *  or timeout.
 * ================================================================== */
bool ESP8266_WaitFor(const char *success, const char *fail, uint32_t timeout) {
  char rx_buf[ESP8266_RX_BUF_SIZE];
  uint16_t rx_idx = 0;
  uint32_t start_time = HAL_GetTick();
  uint8_t ch;

  memset(rx_buf, 0, sizeof(rx_buf));

  while ((HAL_GetTick() - start_time) < timeout) {
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK) {
      if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = ch;
        rx_buf[rx_idx] = '\0';
      }
      /* Check fail FIRST – avoids false positives when the success
       * token is a substring of a longer failure message */
      if (fail && strstr(rx_buf, fail) != NULL) {
#if ESP8266_DEBUG
        printf("[ESP RX] FAIL got \"%s\" | buf: %s\n", fail, rx_buf);
#endif
        return false;
      }
      if (strstr(rx_buf, success) != NULL) {
#if ESP8266_DEBUG
        printf("[ESP RX] OK got \"%s\" | buf: %s\n", success, rx_buf);
#endif
        return true;
      }
    }
  }
#if ESP8266_DEBUG
  printf("[ESP RX] TIMEOUT waiting for \"%s\" (%lums) | buf(%u): %s\n",
         success, (unsigned long)timeout, rx_idx, rx_buf);
#endif
  return false;
}

/* ==================================================================
 *  Low-level: read one line from UART (up to \n or timeout)
 * ================================================================== */
bool ESP8266_ReadLine(char *buf, uint16_t max_len, uint32_t timeout_ms) {
  uint16_t idx = 0;
  uint32_t start = HAL_GetTick();
  uint8_t ch;

  memset(buf, 0, max_len);

  while ((HAL_GetTick() - start) < timeout_ms) {
    if (HAL_UART_Receive(&huart1, &ch, 1, 10) == HAL_OK) {
      if (idx < max_len - 1) {
        buf[idx++] = ch;
        buf[idx] = '\0';
      }
      if (ch == '\n')
        return true;
    }
  }
  return idx > 0;
}

/* ==================================================================
 *  Basic AT Commands
 * ================================================================== */

/* AT – Test AT startup.   Response: OK */
bool ESP8266_Test(void) {
  ESP8266_SendCmd("AT");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+RST – Restart module.  Response: OK … then "ready" after reboot */
bool ESP8266_Reset(void) {
  ESP8266_SendCmd("AT+RST");
  return ESP8266_CheckResponse("ready", 5000); // Wait up to 5s for ready
}

/* ATE0 – Disable command echo (cleaner response parsing) */
bool ESP8266_DisableEcho(void) {
  ESP8266_SendCmd("ATE0");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+GMR – View version info.  Response: <version>\r\nOK */
bool ESP8266_GetVersion(void) {
  ESP8266_SendCmd("AT+GMR");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* ==================================================================
 *  WiFi Commands
 * ================================================================== */

/* AT+CWMODE=<mode> – Set WiFi mode.
 *   1 = Station, 2 = SoftAP, 3 = Station + SoftAP
 *   Response: OK */
bool ESP8266_SetMode(uint8_t mode) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "AT+CWMODE=%u", mode);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CWJAP="<ssid>","<pwd>" – Connect to AP.
 *   Response: WIFI CONNECTED\r\nWIFI GOT IP\r\nOK  (or FAIL)
 *
 *   We wait for "GOT IP" specifically rather than bare "OK", because
 *   with echo enabled the echoed command text or CWMODE's OK can cause
 *   a false-positive match before the join actually completes. */
bool ESP8266_ConnectWiFi(const char *ssid, const char *password) {
  char cmd[128];

  snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
  ESP8266_SendCmd(cmd);

  /* Wait for "GOT IP" – this only appears after a real successful join.
   * FAIL appears on wrong password / unreachable AP. */
  return ESP8266_WaitFor("GOT IP", "FAIL", 20000);
}

/* AT+CWQAP – Disconnect from current AP.  Response: OK */
bool ESP8266_DisconnectWiFi(void) {
  ESP8266_SendCmd("AT+CWQAP");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CWLAP – List available APs.
 *   Response: +CWLAP:<ecn>,<ssid>,<rssi>,<mac>\r\n … OK */
bool ESP8266_ListAPs(void) {
  ESP8266_SendCmd("AT+CWLAP");
  return ESP8266_WaitFor("OK", "ERROR", 10000);
}

/* AT+CWDHCP=<mode>,<en> – Enable/Disable DHCP.
 *   <mode> 1=Station, 2=SoftAP, 3=Both
 *   <en>   0=enable, 1=disable   (note: inverted per ref v1.0)
 *   Response: OK */
bool ESP8266_SetDHCP(uint8_t mode, uint8_t en) {
  char cmd[24];
  snprintf(cmd, sizeof(cmd), "AT+CWDHCP=%u,%u", mode, en);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* ==================================================================
 *  TCP/IP Commands
 * ================================================================== */

/* AT+CIPSTATUS – Get connection status.
 *   Response: STATUS:<stat>\r\n+CIPSTATUS:…\r\nOK */
bool ESP8266_GetStatus(void) {
  ESP8266_SendCmd("AT+CIPSTATUS");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPMUX=<mode> – Set connection multiplexing.
 *   0 = single connection, 1 = multiple connections (up to 4)
 *   Response: OK
 *   Note: must be set before AT+CIPSTART. */
bool ESP8266_SetMux(uint8_t mode) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "AT+CIPMUX=%u", mode);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPSTART="TCP","<ip>",<port> – Start TCP connection (single mode).
 *   Response: OK or ERROR or ALREADY CONNECT */
bool ESP8266_StartTCP(const char *ip, uint16_t port) {
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", ip, port);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 10000);
}

/* AT+CIPSTART=<id>,"TCP","<ip>",<port> – Start TCP (multiple mode). */
bool ESP8266_StartTCPMux(uint8_t id, const char *ip, uint16_t port) {
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=%u,\"TCP\",\"%s\",%u", id, ip, port);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 10000);
}

/* AT+CIPSEND=<length> – Send data (single connection mode).
 *   Waits for ">" prompt, then transmits raw bytes.
 *   Response after data: SEND OK or SEND FAIL */
bool ESP8266_Send(const uint8_t *data, uint16_t len) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", len);
  ESP8266_SendCmd(cmd);

  /* Wait for the ">" prompt indicating the module is ready for data */
  if (!ESP8266_CheckResponse(">", 5000)) {
    return false;
  }

  /* Transmit the raw payload */
  HAL_UART_Transmit(&huart1, (uint8_t *)data, len, HAL_MAX_DELAY);

  return ESP8266_WaitFor("SEND OK", "SEND FAIL", 10000);
}

/* AT+CIPSEND=<id>,<length> – Send data (multiple connection mode). */
bool ESP8266_SendMux(uint8_t id, const uint8_t *data, uint16_t len) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u,%u", id, len);
  ESP8266_SendCmd(cmd);

  if (!ESP8266_CheckResponse(">", 5000)) {
    return false;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)data, len, HAL_MAX_DELAY);

  return ESP8266_WaitFor("SEND OK", "SEND FAIL", 10000);
}

/* Convenience wrapper: send a null-terminated string to a mux client */
bool ESP8266_SendToClient(uint8_t link_id, const char *data) {
  uint16_t len = (uint16_t)strlen(data);
  return ESP8266_SendMux(link_id, (const uint8_t *)data, len);
}

/* AT+CIPCLOSE – Close connection (single mode).  Response: OK */
bool ESP8266_Close(void) {
  ESP8266_SendCmd("AT+CIPCLOSE");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPCLOSE=<id> – Close connection (multiple mode). */
bool ESP8266_CloseMux(uint8_t id) {
  char cmd[20];
  snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%u", id);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIFSR – Get local IP address.  Response: <IP address>\r\nOK */
bool ESP8266_GetIP(void) {
  ESP8266_SendCmd("AT+CIFSR");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPSERVER=1,<port> – Start TCP server on <port>.
 *   Requires AT+CIPMUX=1 first.
 *   Response: OK */
bool ESP8266_StartServer(uint16_t port) {
  char cmd[32];
  snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%u", port);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPSERVER=0 – Stop TCP server.  Response: OK */
bool ESP8266_StopServer(void) {
  ESP8266_SendCmd("AT+CIPSERVER=0");
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPMODE=<mode> – Set transmission mode.
 *   0 = normal mode, 1 = unvarnished (transparent) transmission
 *   Response: OK */
bool ESP8266_SetTransMode(uint8_t mode) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "AT+CIPMODE=%u", mode);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}

/* AT+CIPSTO=<time> – Set TCP server timeout (0-7200 seconds).
 *   Response: OK */
bool ESP8266_SetTimeout(uint16_t seconds) {
  char cmd[20];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTO=%u", seconds);
  ESP8266_SendCmd(cmd);
  return ESP8266_WaitFor("OK", "ERROR", 2000);
}
