#include "stm32f1xx_hal.h"
#include "l298n.h"
#include "hc_sr04.h"
#include "ir_sensor.h"
#include "mpu6050.h"

/* Global Handlers */
L298N_t motor_driver;
HC_SR04_t ultrasonic_sensor;
IR_Sensor_t ir_sensor_left;
IR_Sensor_t ir_sensor_right;
MPU6050_t imu;

/* Forward Declarations */
void SystemClock_Config(void);
void GPIO_Init(void);
void TIM_Init(void);
void I2C_Init(void);
void ADC_Init(void);

/**
 * @brief Main application entry point
 */
int main(void)
{
    /* Initialize HAL */
    HAL_Init();
    SystemClock_Config();
    
    /* Initialize peripherals */
    GPIO_Init();
    TIM_Init();
    I2C_Init();
    ADC_Init();
    
    /* Initialize sensors and actuators */
    // L298N_Init(&motor_driver);
    HC_SR04_Init(&ultrasonic_sensor);
    // IR_Sensor_Init(&ir_sensor_left);
    // IR_Sensor_Init(&ir_sensor_right);
    // MPU6050_Init(&imu);
    
    /* Main loop */
    while (1)
    {
        /* Blink LED on PC13 */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        
        /* Read sensors */
        HC_SR04_StartMeasurement(&ultrasonic_sensor);
        HAL_Delay(50); // Wait for the echo to bounce back
        
        float distance = HC_SR04_GetDistance(&ultrasonic_sensor);
        
        /* Debugging Notes: Place a breakpoint or add 'distance' to your GDB Watch list here! */
        
        HAL_Delay(100);
    }
    
    return 0;
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief System Clock Configuration (Configure for your STM32 variant)
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while(1);
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | 
                                   RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        while(1);
    }
}

/**
 * @brief GPIO Initialization
 * Configure GPIO pins for motor control and sensors
 */
void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    
    /* Configure motor driver pins */
    motor_driver.motor_left.IN1_PORT = GPIOA;
    motor_driver.motor_left.IN1_PIN = GPIO_PIN_0;
    motor_driver.motor_left.IN2_PORT = GPIOA;
    motor_driver.motor_left.IN2_PIN = GPIO_PIN_1;
    
    motor_driver.motor_right.IN1_PORT = GPIOA;
    motor_driver.motor_right.IN1_PIN = GPIO_PIN_2;
    motor_driver.motor_right.IN2_PORT = GPIOA;
    motor_driver.motor_right.IN2_PIN = GPIO_PIN_3;
    
    /* Configure ultrasonic sensor pins */
    ultrasonic_sensor.TRIG_PORT = GPIOB;
    ultrasonic_sensor.TRIG_PIN = GPIO_PIN_0;
    ultrasonic_sensor.ECHO_PORT = GPIOB;
    ultrasonic_sensor.ECHO_PIN = GPIO_PIN_1;
    
    /* Configure IR sensor pins */
    ir_sensor_left.GPIO_PORT = GPIOC;
    ir_sensor_left.GPIO_PIN = GPIO_PIN_0;
    ir_sensor_left.mode = IR_MODE_DIGITAL;
    
    ir_sensor_right.GPIO_PORT = GPIOC;
    ir_sensor_right.GPIO_PIN = GPIO_PIN_1;
    ir_sensor_right.mode = IR_MODE_DIGITAL;

    /* Configure PC13 for onboard LED */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/**
 * @brief Timer Initialization
 * Configure timers for PWM (motors) and input capture (ultrasonic)
 */
void TIM_Init(void)
{
    /* Configure TIM3 for HC-SR04 (1 MHz timer, 1us per tick) */
    static TIM_HandleTypeDef htim3;
    TIM_IC_InitTypeDef sConfigIC = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 64 - 1; // Assuming 64MHz System Clock -> 1MHz timer clock
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xFFFF;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_IC_Init(&htim3);

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING; // STM32F1 can't do BOTHEDGE natively!
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_4); // PB1 is TIM3_CH4
    
    ultrasonic_sensor.htim = &htim3;
    ultrasonic_sensor.tim_channel = TIM_CHANNEL_4;

    /* Start the timer counter */
    HAL_TIM_Base_Start(&htim3);

    /* Enable TIM3 global interrupt for Input Capture */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    /* PWM limits for L298N (for later) */
    motor_driver.motor_right.PWM_Max = 1000;
}

/**
 * @brief I2C Initialization
 * Configure I2C for MPU6050 communication
 */
void I2C_Init(void)
{
    /* This would be configured via CubeMX */
    /* I2C1 or I2C2 for MPU6050 */
    
    imu.i2c_addr = MPU6050_ADDR;
}

/**
 * @brief ADC Initialization
 * Configure ADC for analog IR sensors
 */
void ADC_Init(void)
{
    /* This would be configured via CubeMX */
    /* ADC for analog IR sensors if needed */
}

/**
 * @brief Timer Input Capture Interrupt Handler
 * Call this from HAL_TIM_IC_CaptureCallback() in stm32fxx_it.c
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)  /* Adjust based on your configuration */
    {
        HC_SR04_Echo_Callback(&ultrasonic_sensor);
    }
}

/**
 * @brief External Interrupt Handler
 * Call from HAL_GPIO_EXTI_Callback() in stm32fxx_it.c for line detection
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_0)  /* IR Left sensor */
    {
        /* Handle left IR detection */
    }
    else if (GPIO_Pin == GPIO_PIN_1)  /* IR Right sensor */
    {
        /* Handle right IR detection */
    }
}

/**
 * @brief TIM3 Interrupt Handler
 */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(ultrasonic_sensor.htim);
}
