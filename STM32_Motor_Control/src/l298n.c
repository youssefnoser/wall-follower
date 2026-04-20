#include "l298n.h"

/**
 * @brief Initialize L298N Motor Driver
 * @param h_motor: L298N handler
 */
void L298N_Init(L298N_t *h_motor)
{
    /* Initialize left motor GPIO (IN1, IN2) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    /* Configure IN1 and IN2 pins as outputs */
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    GPIO_InitStruct.Pin = h_motor->motor_left.IN1_PIN;
    HAL_GPIO_Init(h_motor->motor_left.IN1_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = h_motor->motor_left.IN2_PIN;
    HAL_GPIO_Init(h_motor->motor_left.IN2_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = h_motor->motor_right.IN1_PIN;
    HAL_GPIO_Init(h_motor->motor_right.IN1_PORT, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = h_motor->motor_right.IN2_PIN;
    HAL_GPIO_Init(h_motor->motor_right.IN2_PORT, &GPIO_InitStruct);
    
    /* Stop all motors */
    L298N_StopAll(h_motor);
}

/**
 * @brief Set motor speed using PWM
 * @param h_motor: L298N handler
 * @param motor: MOTOR_LEFT or MOTOR_RIGHT
 * @param speed: Speed value (0-100)
 */
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

/**
 * @brief Set motor direction
 * @param h_motor: L298N handler
 * @param motor: MOTOR_LEFT or MOTOR_RIGHT
 * @param direction: MOTOR_FORWARD or MOTOR_BACKWARD
 */
void L298N_SetDirection(L298N_t *h_motor, uint8_t motor, uint8_t direction)
{
    L298N_Motor_t *motor_ptr;
    
    motor_ptr = (motor == MOTOR_LEFT) ? &h_motor->motor_left : &h_motor->motor_right;
    
    if (direction == MOTOR_FORWARD)
    {
        HAL_GPIO_WritePin(motor_ptr->IN1_PORT, motor_ptr->IN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor_ptr->IN2_PORT, motor_ptr->IN2_PIN, GPIO_PIN_RESET);
    }
    else if (direction == MOTOR_BACKWARD)
    {
        HAL_GPIO_WritePin(motor_ptr->IN1_PORT, motor_ptr->IN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor_ptr->IN2_PORT, motor_ptr->IN2_PIN, GPIO_PIN_SET);
    }
}

/**
 * @brief Move motor with direction and speed
 * @param h_motor: L298N handler
 * @param motor: MOTOR_LEFT or MOTOR_RIGHT
 * @param direction: MOTOR_FORWARD or MOTOR_BACKWARD
 * @param speed: Speed value (0-100)
 */
void L298N_Move(L298N_t *h_motor, uint8_t motor, uint8_t direction, uint8_t speed)
{
    L298N_SetDirection(h_motor, motor, direction);
    L298N_SetSpeed(h_motor, motor, speed);
}

/**
 * @brief Stop a single motor
 * @param h_motor: L298N handler
 * @param motor: MOTOR_LEFT or MOTOR_RIGHT
 */
void L298N_Stop(L298N_t *h_motor, uint8_t motor)
{
    L298N_SetSpeed(h_motor, motor, 0);
}

/**
 * @brief Stop all motors
 * @param h_motor: L298N handler
 */
void L298N_StopAll(L298N_t *h_motor)
{
    L298N_Stop(h_motor, MOTOR_LEFT);
    L298N_Stop(h_motor, MOTOR_RIGHT);
}
