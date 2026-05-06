#include "ir_sensor.h"

/**
 * @brief Initialize IR Sensor
 * @param ir_sensor: IR sensor handler
 */
void IR_Sensor_Init(IR_Sensor_t *ir_sensor)
{
    if (ir_sensor->mode == IR_MODE_DIGITAL)
    {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        
        /* Configure GPIO pin as input */
        GPIO_InitStruct.Pin = ir_sensor->GPIO_PIN;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(ir_sensor->GPIO_PORT, &GPIO_InitStruct);
    }
    else if (ir_sensor->mode == IR_MODE_ANALOG)
    {
        /* ADC is configured via CubeMX, just need to start it */
        if (HAL_ADC_Start(ir_sensor->hadc) != HAL_OK)
        {
            /* Error handling */
        }
    }
    
    ir_sensor->state = IR_NO_OBJECT;
}

/**
 * @brief Set IR sensor mode
 * @param ir_sensor: IR sensor handler
 * @param mode: IR_MODE_DIGITAL or IR_MODE_ANALOG
 */
void IR_Sensor_SetMode(IR_Sensor_t *ir_sensor, uint8_t mode)
{
    ir_sensor->mode = mode;
    IR_Sensor_Init(ir_sensor);
}

/**
 * @brief Get IR sensor state (digital mode)
 * @param ir_sensor: IR sensor handler
 * @return IR_OBJECT_DETECTED or IR_NO_OBJECT
 */
uint8_t IR_Sensor_GetState(IR_Sensor_t *ir_sensor)
{
    if (ir_sensor->mode == IR_MODE_DIGITAL)
    {
        /* Active-low module: pin LOW = object detected, pin HIGH = clear. */
        GPIO_PinState pin = HAL_GPIO_ReadPin(ir_sensor->GPIO_PORT, ir_sensor->GPIO_PIN);
        ir_sensor->state = (pin == GPIO_PIN_RESET) ? IR_OBJECT_DETECTED : IR_NO_OBJECT;
        return ir_sensor->state;
    }
    
    return IR_NO_OBJECT;
}

/**
 * @brief Get analog value from IR sensor (analog mode)
 * @param ir_sensor: IR sensor handler
 * @return ADC value (0-4095 for 12-bit ADC)
 */
uint16_t IR_Sensor_GetAnalogValue(IR_Sensor_t *ir_sensor)
{
    if (ir_sensor->mode == IR_MODE_ANALOG && ir_sensor->hadc != NULL)
    {
        if (HAL_ADC_PollForConversion(ir_sensor->hadc, 100) == HAL_OK)
        {
            return (uint16_t)HAL_ADC_GetValue(ir_sensor->hadc);
        }
    }
    
    return 0;
}

/**
 * @brief Set threshold for analog IR sensor
 * @param ir_sensor: IR sensor handler
 * @param threshold: Threshold value (0-4095)
 */
void IR_Sensor_SetThreshold(IR_Sensor_t *ir_sensor, uint16_t threshold)
{
    if (threshold > 4095)
        threshold = 4095;
    
    ir_sensor->threshold = threshold;
}
