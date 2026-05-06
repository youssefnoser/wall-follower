#ifndef	_WIFICONFIG_H
#define	_WIFICONFIG_H

/*
 * Project-specific configuration for the jrmejiaa ESP8266 driver.
 * USART1 (PA9 TX / PA10 RX) is initialised in main.c at 115200-8-N-1.
 *
 * The CH_PD/RST pins below are routed to PC14/PC15, which are unused on
 * the Blue Pill when HSE is not configured. Wifi_Enable / Wifi_Disable
 * are not strictly required (CH_PD is normally tied high externally),
 * but we point them at safe pins so the GPIO writes have no side effect.
 */
#define 	_WIFI_USART  					huart1
#define		_WIFI_RX_SIZE					512
#define		_WIFI_RX_FOR_DATA_SIZE			1024
#define		_WIFI_TX_SIZE					256
#define		_WIFI_TASK_SIZE					512
#define 	_MAX_SEND_BYTES					2048
// Buttons of the ESP8266 Module (unused safe pins on Blue Pill)
#define		_BANK_WIFI_BUTTONS				GPIOC
#define		_BUTTON_RST						GPIO_PIN_15
#define		_BUTTON_ENABLE					GPIO_PIN_14
// The times are defined in ms
#define		_WIFI_WAIT_TIME_LOW				1000
#define		_WIFI_WAIT_TIME_MED				5000
#define		_WIFI_WAIT_TIME_HIGH			15000
#define		_WIFI_WAIT_TIME_VERYHIGH		35000

#endif
