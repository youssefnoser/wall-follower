#ifndef HC_SR04_H
#define HC_SR04_H

#include "stm32f1xx_hal.h"

/* HC-SR04 Configuration */
#define HC_SR04_TIMEOUT      30000  /* 30ms timeout */
#define HC_SR04_MEDIAN_SIZE  3      /* median window (odd number) */
#define HC_SR04_EMA_ALPHA    0.7f   /* EMA smoothing factor (0=heavy smooth, 1=no smooth) */

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
    float distance_cm;             /* raw distance */
    uint8_t filter_enabled;        /* 1 = use filtered_cm, 0 = use distance_cm */

    /* Filter state */
    float    median_buf[HC_SR04_MEDIAN_SIZE];
    uint8_t  median_idx;
    uint8_t  median_count;
    float    ema_value;
    float    filtered_cm;          /* median + EMA filtered output */
} HC_SR04_t;

/* Function Prototypes */
void  HC_SR04_Init(HC_SR04_t *hc_sr04);
void  HC_SR04_Trigger(HC_SR04_t *hc_sr04);
void  HC_SR04_Echo_Callback(HC_SR04_t *hc_sr04);
float HC_SR04_GetDistance(HC_SR04_t *hc_sr04);      /* returns filtered or raw based on filter_enabled flag */
float HC_SR04_GetRawDistance(HC_SR04_t *hc_sr04);   /* strictly unfiltered */
void  HC_SR04_StartMeasurement(HC_SR04_t *hc_sr04);
void  HC_SR04_SetFilterEnabled(HC_SR04_t *hc_sr04, uint8_t enable);

#endif /* HC_SR04_H */
