#ifndef L298N_H
#define L298N_H

#include "stm32f1xx_hal.h"

/* Motor Definitions */
#define MOTOR_LEFT          0
#define MOTOR_RIGHT         1

/* Motor Speed Range */
#define MOTOR_SPEED_MIN     0
#define MOTOR_SPEED_MAX     100

/* Motor Direction */
#define MOTOR_FORWARD       1
#define MOTOR_BACKWARD      0
#define MOTOR_STOP          2

/* L298N Pin Configuration */
typedef struct {
    GPIO_TypeDef *IN1_PORT;
    uint16_t IN1_PIN;
    GPIO_TypeDef *IN2_PORT;
    uint16_t IN2_PIN;
    TIM_HandleTypeDef *htim;
    uint32_t PWM_Channel;
    uint32_t PWM_Max;
} L298N_Motor_t;

typedef struct {
    L298N_Motor_t motor_left;
    L298N_Motor_t motor_right;
} L298N_t;

/* Function Prototypes */
void L298N_Init(L298N_t *h_motor);
void L298N_SetSpeed(L298N_t *h_motor, uint8_t motor, uint8_t speed);
void L298N_SetDirection(L298N_t *h_motor, uint8_t motor, uint8_t direction);
void L298N_Move(L298N_t *h_motor, uint8_t motor, uint8_t direction, uint8_t speed);
void L298N_Stop(L298N_t *h_motor, uint8_t motor);
void L298N_StopAll(L298N_t *h_motor);

#endif /* L298N_H */
