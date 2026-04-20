#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "stm32f1xx_hal.h"

/* IR Sensor Modes */
#define IR_MODE_DIGITAL   0  /* GPIO Input */
#define IR_MODE_ANALOG    1  /* ADC Input */

/* IR Sensor State */
#define IR_OBJECT_DETECTED   1
#define IR_NO_OBJECT         0

typedef struct {
    GPIO_TypeDef *GPIO_PORT;
    uint16_t GPIO_PIN;
    ADC_HandleTypeDef *hadc;
    uint32_t ADC_CHANNEL;
    uint8_t mode;
    uint16_t threshold;  /* For analog mode */
    uint8_t state;
} IR_Sensor_t;

/* Function Prototypes */
void IR_Sensor_Init(IR_Sensor_t *ir_sensor);
void IR_Sensor_SetMode(IR_Sensor_t *ir_sensor, uint8_t mode);
uint8_t IR_Sensor_GetState(IR_Sensor_t *ir_sensor);
uint16_t IR_Sensor_GetAnalogValue(IR_Sensor_t *ir_sensor);
void IR_Sensor_SetThreshold(IR_Sensor_t *ir_sensor, uint16_t threshold);

#endif /* IR_SENSOR_H */
