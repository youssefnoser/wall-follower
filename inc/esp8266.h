#ifndef _ESP8266_H_
#define _ESP8266_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

extern UART_HandleTypeDef huart2;

void ESP8266_Init(void);
void ESP8266_SendCmd(const char *cmd);
bool ESP8266_CheckResponse(const char *expected_response, uint32_t timeout);
bool ESP8266_ConnectWiFi(const char *ssid, const char *password);
bool ESP8266_Reset(void);

#endif /* _ESP8266_H_ */
