#ifndef _ESP8266_H_
#define _ESP8266_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Maximum response buffer size */
#define ESP8266_RX_BUF_SIZE 512

extern UART_HandleTypeDef huart2;

/* ---- Low-level --------------------------------------------------- */
void ESP8266_Init(void);
void ESP8266_SendCmd(const char *cmd);
bool ESP8266_CheckResponse(const char *expected_response, uint32_t timeout);
bool ESP8266_WaitFor(const char *success, const char *fail, uint32_t timeout);
bool ESP8266_ReadLine(char *buf, uint16_t max_len, uint32_t timeout_ms);

/* ---- Basic AT commands ------------------------------------------- */
bool ESP8266_Test(void);              /* AT            */
bool ESP8266_Reset(void);             /* AT+RST        */
bool ESP8266_DisableEcho(void);       /* ATE0          */
bool ESP8266_GetVersion(void);        /* AT+GMR        */

/* ---- WiFi commands ----------------------------------------------- */
bool ESP8266_SetMode(uint8_t mode);   /* AT+CWMODE=<mode> (1/2/3) */
bool ESP8266_ConnectWiFi(const char *ssid, const char *password); /* AT+CWJAP */
bool ESP8266_DisconnectWiFi(void);    /* AT+CWQAP      */
bool ESP8266_ListAPs(void);           /* AT+CWLAP      */
bool ESP8266_SetDHCP(uint8_t mode, uint8_t en); /* AT+CWDHCP */

/* ---- TCP/IP commands --------------------------------------------- */
bool ESP8266_GetStatus(void);         /* AT+CIPSTATUS   */
bool ESP8266_SetMux(uint8_t mode);    /* AT+CIPMUX=<mode> (0/1) */
bool ESP8266_StartTCP(const char *ip, uint16_t port);  /* AT+CIPSTART (single) */
bool ESP8266_StartTCPMux(uint8_t id, const char *ip, uint16_t port); /* (mux) */
bool ESP8266_Send(const uint8_t *data, uint16_t len);  /* AT+CIPSEND (single)  */
bool ESP8266_SendMux(uint8_t id, const uint8_t *data, uint16_t len); /* (mux)  */
bool ESP8266_SendToClient(uint8_t link_id, const char *data); /* convenience */
bool ESP8266_Close(void);             /* AT+CIPCLOSE (single) */
bool ESP8266_CloseMux(uint8_t id);    /* AT+CIPCLOSE=<id>     */
bool ESP8266_GetIP(void);             /* AT+CIFSR      */
bool ESP8266_StartServer(uint16_t port);  /* AT+CIPSERVER=1,<port> */
bool ESP8266_StopServer(void);        /* AT+CIPSERVER=0 */
bool ESP8266_SetTransMode(uint8_t mode);  /* AT+CIPMODE=<mode>   */
bool ESP8266_SetTimeout(uint16_t seconds); /* AT+CIPSTO=<time>   */

#endif /* _ESP8266_H_ */
