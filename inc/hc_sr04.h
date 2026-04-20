#ifndef HC_SR04_H
#define HC_SR04_H

#include "stm32f1xx_hal.h"

/* HC-SR04 Configuration */
#define HC_SR04_TIMEOUT 30000  /* 30ms timeout */

typedef struct {
    GPIO_TypeDef *TRIG_PORT;
    uint16_t TRIG_PIN;
    GPIO_TypeDef *ECHO_PORT;
    uint16_t ECHO_PIN;
    TIM_HandleTypeDef *htim;
    uint32_t tim_channel;
    uint32_t first_capture;
    uint32_t second_capture;
    uint8_t is_first_capture;
    float distance_cm;
} HC_SR04_t;

/* Function Prototypes */
void HC_SR04_Init(HC_SR04_t *hc_sr04);
void HC_SR04_Trigger(HC_SR04_t *hc_sr04);
void HC_SR04_Echo_Callback(HC_SR04_t *hc_sr04);
float HC_SR04_GetDistance(HC_SR04_t *hc_sr04);
void HC_SR04_StartMeasurement(HC_SR04_t *hc_sr04);

#endif /* HC_SR04_H */
