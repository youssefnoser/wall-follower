#include "esp8266.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart2;

void ESP8266_Init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  // PA2 TX -> ESP RX
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // PA3 RX -> ESP TX
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200; // Standard default baud rate for ESP-01
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);
}

void ESP8266_SendCmd(const char *cmd) {
  HAL_UART_Transmit(&huart2, (uint8_t *)cmd, strlen(cmd), HAL_MAX_DELAY);
  HAL_UART_Transmit(&huart2, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY);
}

bool ESP8266_CheckResponse(const char *expected_response, uint32_t timeout) {
  char rx_buf[256];
  uint16_t rx_idx = 0;
  uint32_t start_time = HAL_GetTick();
  uint8_t ch;

  memset(rx_buf, 0, sizeof(rx_buf));

  while ((HAL_GetTick() - start_time) < timeout) {
    if (HAL_UART_Receive(&huart2, &ch, 1, 10) == HAL_OK) {
      if (rx_idx < sizeof(rx_buf) - 1) {
        rx_buf[rx_idx++] = ch;
        rx_buf[rx_idx] = '\0'; // Null terminate
      }
      if (strstr(rx_buf, expected_response) != NULL) {
        return true;
      }
    }
  }
  return false;
}

bool ESP8266_Reset(void) {
  ESP8266_SendCmd("AT+RST");
  return ESP8266_CheckResponse("ready", 5000); // Wait up to 5s for ready
}

bool ESP8266_ConnectWiFi(const char *ssid, const char *password) {
  char cmd[128];

  ESP8266_SendCmd("AT+CWMODE=1"); // Set station mode
  if (!ESP8266_CheckResponse("OK", 2000)) {
    return false;
  }

  snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid, password);
  ESP8266_SendCmd(cmd);

  // Connecting to WiFi can take an extended time
  if (ESP8266_CheckResponse("WIFI CONNECTED", 15000) &&
      ESP8266_CheckResponse("WIFI GOT IP", 10000)) {
    return ESP8266_CheckResponse("OK", 2000);
  }

  return false;
}
