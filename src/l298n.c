#include "l298n.h"

/* Initialize L298N Motor Driver */
void L298N_Init(L298N_t *h_motor)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    /* IN pins only - EN/PWM pins are owned by TIM_Init */
    GPIO_InitStruct.Pin = h_motor->motor_left.IN1_PIN;
    HAL_GPIO_Init(h_motor->motor_left.IN1_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = h_motor->motor_left.IN2_PIN;
    HAL_GPIO_Init(h_motor->motor_left.IN2_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = h_motor->motor_right.IN1_PIN;
    HAL_GPIO_Init(h_motor->motor_right.IN1_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = h_motor->motor_right.IN2_PIN;
    HAL_GPIO_Init(h_motor->motor_right.IN2_PORT, &GPIO_InitStruct);


    L298N_StopAll(h_motor);
}

/* Set motor speed (0-100) */
void L298N_SetSpeed(L298N_t *h_motor, uint8_t motor, uint8_t speed)
{
    L298N_Motor_t *motor_ptr;
    
    if (speed > 100)
        speed = 100;
    
    motor_ptr = (motor == MOTOR_LEFT) ? &h_motor->motor_left : &h_motor->motor_right;
    
    uint32_t pwm_value = (speed * motor_ptr->PWM_Max) / 100;
    
    if (motor_ptr->htim != NULL)
    {
        __HAL_TIM_SET_COMPARE(motor_ptr->htim, motor_ptr->PWM_Channel, pwm_value);
    }
}

/* Set motor direction */
void L298N_SetDirection(L298N_t *h_motor, uint8_t motor, uint8_t direction)
{
    L298N_Motor_t *motor_ptr =
        (motor == MOTOR_LEFT) ? &h_motor->motor_left : &h_motor->motor_right;

    if (motor == MOTOR_RIGHT) {
        direction = (direction == MOTOR_FORWARD) ? MOTOR_BACKWARD : MOTOR_FORWARD;
    }

    if (direction == MOTOR_FORWARD) {
        HAL_GPIO_WritePin(motor_ptr->IN1_PORT, motor_ptr->IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->IN2_PORT, motor_ptr->IN2_PIN, GPIO_PIN_RESET);
    } else if (direction == MOTOR_BACKWARD) {
        HAL_GPIO_WritePin(motor_ptr->IN1_PORT, motor_ptr->IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->IN2_PORT, motor_ptr->IN2_PIN, GPIO_PIN_SET);
    }
}

/* Move motor */
void L298N_Move(L298N_t *h_motor, uint8_t motor, uint8_t direction, uint8_t speed)
{
    L298N_SetDirection(h_motor, motor, direction);
    L298N_SetSpeed(h_motor, motor, speed);
}

/* Stop a single motor */
void L298N_Stop(L298N_t *h_motor, uint8_t motor)
{
    L298N_Motor_t *motor_ptr =
        (motor == MOTOR_LEFT) ? &h_motor->motor_left : &h_motor->motor_right;

    /* Coast: both inputs LOW. Works whether EN is PWM-driven or jumpered high. */
    HAL_GPIO_WritePin(motor_ptr->IN1_PORT, motor_ptr->IN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor_ptr->IN2_PORT, motor_ptr->IN2_PIN, GPIO_PIN_RESET);
    L298N_SetSpeed(h_motor, motor, 0);
}

/* Stop all motors */
void L298N_StopAll(L298N_t *h_motor)
{
    L298N_Stop(h_motor, MOTOR_LEFT);
    L298N_Stop(h_motor, MOTOR_RIGHT);
}
