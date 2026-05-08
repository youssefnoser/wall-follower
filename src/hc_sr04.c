#include "hc_sr04.h"
#include <string.h>

/**
 * @brief Initialize HC-SR04 Ultrasonic Sensor
 * @param hc_sr04: HC-SR04 handler
 */
void HC_SR04_Init(HC_SR04_t *hc_sr04)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure TRIG pin as output */
    GPIO_InitStruct.Pin = hc_sr04->TRIG_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(hc_sr04->TRIG_PORT, &GPIO_InitStruct);
    
    /* Configure ECHO pin for Timer Input Capture */
    GPIO_InitStruct.Pin = hc_sr04->ECHO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Standard input for F1 Timer Input
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hc_sr04->ECHO_PORT, &GPIO_InitStruct);
    
    /* Initialize variables */
    hc_sr04->is_first_capture = 1;
    hc_sr04->distance_cm = 0;

    /* Initialize filter state */
    memset(hc_sr04->median_buf, 0, sizeof(hc_sr04->median_buf));
    hc_sr04->median_idx   = 0;
    hc_sr04->median_count = 0;
    hc_sr04->ema_value    = 0.0f;
    hc_sr04->filtered_cm  = 0.0f;

    /* Set TRIG to LOW */
    HAL_GPIO_WritePin(hc_sr04->TRIG_PORT, hc_sr04->TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Send trigger pulse to HC-SR04
 * @param hc_sr04: HC-SR04 handler
 */
void HC_SR04_Trigger(HC_SR04_t *hc_sr04)
{
    /* Send 10us pulse */
    HAL_GPIO_WritePin(hc_sr04->TRIG_PORT, hc_sr04->TRIG_PIN, GPIO_PIN_SET);
    __HAL_TIM_SET_COUNTER(hc_sr04->htim, 0);
    while(__HAL_TIM_GET_COUNTER(hc_sr04->htim) < 10); /* 10us delay */
    HAL_GPIO_WritePin(hc_sr04->TRIG_PORT, hc_sr04->TRIG_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Start measurement (trigger + start input capture)
 * @param hc_sr04: HC-SR04 handler
 */
void HC_SR04_StartMeasurement(HC_SR04_t *hc_sr04)
{
    hc_sr04->is_first_capture = 1;
    /* Ensure we are looking for a rising edge at the start of a measurement */
    __HAL_TIM_SET_CAPTUREPOLARITY(hc_sr04->htim, hc_sr04->tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
    
    HAL_TIM_IC_Start_IT(hc_sr04->htim, hc_sr04->tim_channel);
    HC_SR04_Trigger(hc_sr04);
}

/* Apply median-then-EMA filter to the latest raw sample. Called internally. */
static void HC_SR04_UpdateFilter(HC_SR04_t *hc_sr04)
{
    /* --- Median stage: circular buffer of last MEDIAN_SIZE samples --- */
    hc_sr04->median_buf[hc_sr04->median_idx] = hc_sr04->distance_cm;
    hc_sr04->median_idx = (hc_sr04->median_idx + 1) % HC_SR04_MEDIAN_SIZE;
    if (hc_sr04->median_count < HC_SR04_MEDIAN_SIZE)
        hc_sr04->median_count++;

    float tmp[HC_SR04_MEDIAN_SIZE];
    uint8_t n = hc_sr04->median_count;
    memcpy(tmp, hc_sr04->median_buf, n * sizeof(float));

    /* Insertion sort on the n valid samples */
    for (uint8_t i = 1; i < n; i++) {
        float key = tmp[i];
        int8_t j  = (int8_t)(i - 1);
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    float median = tmp[n / 2];

    /* --- EMA stage --- */
    if (hc_sr04->median_count == 1)
        hc_sr04->ema_value = median;  /* bootstrap with first real sample */
    else
        hc_sr04->ema_value = HC_SR04_EMA_ALPHA * median
                           + (1.0f - HC_SR04_EMA_ALPHA) * hc_sr04->ema_value;

    hc_sr04->filtered_cm = hc_sr04->ema_value;
}

/**
 * @brief Handle echo pin state change (called from interrupt handler)
 * @param hc_sr04: HC-SR04 handler
 */
void HC_SR04_Echo_Callback(HC_SR04_t *hc_sr04)
{
    if (hc_sr04->is_first_capture)
    {
        /* Rising edge - start counting */
        hc_sr04->first_capture = HAL_TIM_ReadCapturedValue(hc_sr04->htim, hc_sr04->tim_channel);
        hc_sr04->is_first_capture = 0;
        
        /* Change polarity to FALLING edge to catch the end of the pulse! */
        __HAL_TIM_SET_CAPTUREPOLARITY(hc_sr04->htim, hc_sr04->tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
    }
    else
    {
        /* Falling edge - calculate distance */
        hc_sr04->second_capture = HAL_TIM_ReadCapturedValue(hc_sr04->htim, hc_sr04->tim_channel);
        
        /* Stop interrupt temporarily */
        HAL_TIM_IC_Stop_IT(hc_sr04->htim, hc_sr04->tim_channel);
        
        uint32_t distance_count;
        if(hc_sr04->second_capture >= hc_sr04->first_capture) {
            distance_count = hc_sr04->second_capture - hc_sr04->first_capture;
        } else {
            /* Handle timer wrap-around (since it's a 16-bit timer max 0xFFFF) */
            distance_count = (0xFFFF - hc_sr04->first_capture) + hc_sr04->second_capture + 1;
        }
        
        /* Distance = (time in microseconds) / 58 */
        hc_sr04->distance_cm = distance_count / 58.0f;

        HC_SR04_UpdateFilter(hc_sr04);
    }
}

/**
 * @brief Get filtered distance in cm (median + EMA)
 * @param hc_sr04: HC-SR04 handler
 * @return Filtered distance in centimeters
 */
float HC_SR04_GetDistance(HC_SR04_t *hc_sr04)
{
    return hc_sr04->filtered_cm;
}

/**
 * @brief Get raw (unfiltered) distance in cm
 * @param hc_sr04: HC-SR04 handler
 * @return Raw distance in centimeters
 */
float HC_SR04_GetRawDistance(HC_SR04_t *hc_sr04)
{
    return hc_sr04->distance_cm;
}
