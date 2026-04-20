#include "hc_sr04.h"

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
    }
}

/**
 * @brief Get measured distance in cm
 * @param hc_sr04: HC-SR04 handler
 * @return Distance in centimeters
 */
float HC_SR04_GetDistance(HC_SR04_t *hc_sr04)
{
    return hc_sr04->distance_cm;
}
