# STM32 Embedded Motor Control System

Modular code for controlling a robot with L298N motor driver, HC-SR04 ultrasonic sensor, IR sensors, and MPU6050 IMU.

## Project Structure

```
STM32_Motor_Control/
├── Inc/                      # Header files
│   ├── l298n.h              # L298N Motor Driver
│   ├── hc_sr04.h            # HC-SR04 Ultrasonic Sensor
│   ├── ir_sensor.h          # IR Sensor Module
│   └── mpu6050.h            # MPU6050 IMU Sensor
├── Src/                      # Source files
│   ├── main.c               # Main application
│   ├── l298n.c              # Motor driver implementation
│   ├── hc_sr04.c            # Ultrasonic sensor implementation
│   ├── ir_sensor.c          # IR sensor implementation
│   └── mpu6050.c            # IMU implementation
├── Drivers/                  # HAL drivers (from CubeMX)
└── README.md                # This file
```

## Hardware Connections

### L298N Motor Driver

- **IN1-IN4 (dir)**: PB12-PB15
- **EN (PWM)**: PA0-PA1

### HC-SR04 Ultrasonic Sensor

- **Front**: Trig: PB8, Echo: PA6 (TIM3_CH1)
- **Left**: Trig: PA7, Echo: PB9 (TIM4_CH4)
- **Right**: Trig: PB10, Echo: PB0 (TIM3_CH3)

### MPU6050 IMU

- **I2C**: PB6, PB7

### ESP8266 UART

- **UART**: PA9, PA10

### Built-in LED

- **LED**: PC13

## ESP8266 WIFI

- PA2 TX -> ESP RX
- PA3 RX -> ESP TX

## Module Usage

### Motor Driver (L298N)

```c
L298N_t motor_driver;

// Initialize
L298N_Init(&motor_driver);

// Move motor forward at 70% speed
L298N_Move(&motor_driver, MOTOR_LEFT, MOTOR_FORWARD, 70);
L298N_Move(&motor_driver, MOTOR_RIGHT, MOTOR_FORWARD, 70);

// Stop motor
L298N_Stop(&motor_driver, MOTOR_LEFT);

// Stop all motors
L298N_StopAll(&motor_driver);
```

### Ultrasonic Sensor (HC-SR04)

```c
HC_SR04_t ultrasonic_sensor;

// Initialize
HC_SR04_Init(&ultrasonic_sensor);

// Start measurement
HC_SR04_StartMeasurement(&ultrasonic_sensor);

// Get distance (in callback or after measurement complete)
float distance = HC_SR04_GetDistance(&ultrasonic_sensor);
```

### IR Sensors

```c
IR_Sensor_t ir_sensor_left;

// Initialize
ir_sensor_left.mode = IR_MODE_DIGITAL;  // or IR_MODE_ANALOG
IR_Sensor_Init(&ir_sensor_left);

// Get digital state
uint8_t state = IR_Sensor_GetState(&ir_sensor_left);
if (state == IR_OBJECT_DETECTED) {
    // Object detected
}

// Get analog value (if in analog mode)
uint16_t adc_value = IR_Sensor_GetAnalogValue(&ir_sensor_left);
```

### IMU Sensor (MPU6050)

```c
MPU6050_t imu;

// Initialize
MPU6050_Init(&imu);

// Read all sensors
MPU6050_ReadAll(&imu);

// Access data
float accel_x = imu.accel.x;  // in g
float gyro_z = imu.gyro.z;    // in deg/s
float temp = imu.temperature; // in Celsius

// Or read individual sensors
MPU6050_ReadAccelerometer(&imu);
MPU6050_ReadGyroscope(&imu);
MPU6050_ReadTemperature(&imu);
```

## STM32 CubeMX Configuration

### Timers

1. **TIM2**: PWM for L298N motors
   - CH1: PA5 (Left motor PWM)
   - CH2: PA6 (Right motor PWM)
   - Frequency: 20 kHz

2. **TIM3**: Input Capture for HC-SR04
   - CH2: PB1 (Echo input)
   - Frequency: 1 MHz
   - Enable interrupts

### I2C

- **I2C2**: For MPU6050
  - SCL: PB10
  - SDA: PB11
  - Speed: 400 kHz (Standard mode)

### ADC (if using analog IR sensors)

- **ADC1**:
  - Channel 10: PC0 (Left IR)
  - Channel 11: PC1 (Right IR)
  - Continuous conversion mode

### GPIO

- PA0-PA3: Digital outputs (L298N direction control)
- PB0: Digital output (Ultrasonic TRIG)
- PB1: Input capture (Ultrasonic ECHO)
- PC0-PC1: Digital inputs or ADC inputs (IR sensors)

## Key Features

✓ **Modular Design**: Each component is independent and can be used separately
✓ **HAL Based**: Compatible with STM32CubeMX generated code
✓ **Non-blocking**: Asynchronous sensor readings where applicable
✓ **Error Handling**: Basic error checking in I2C and ADC operations
✓ **Flexible IR Modes**: Digital or analog operation modes
✓ **Complete IMU Support**: Accelerometer, gyroscope, and temperature readings

## Main Application Logic

The example in `main.c` implements a simple obstacle avoidance robot:

1. **Reads ultrasonic sensor** to detect obstacles ahead
2. **Reads IR sensors** to detect edge/line
3. **Reads IMU** for orientation data
4. **Controls motors** based on sensor inputs:
   - Stop and backup if obstacle detected (< 20cm)
   - Turn away if line detected
   - Move forward normally

## Integration Steps

1. Create STM32 project in CubeMX for your MCU
2. Configure peripherals as specified above
3. Copy all files from `Inc/` and `Src/` folders
4. Include headers in your main.c
5. Modify main.c initialization to match your pin configuration
6. Call interrupt handlers from stm32fxx_it.c:
   ```c
   void TIM3_IRQHandler(void) {
       HAL_TIM_IRQHandler(&htim3);
   }
   void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
       // Calls ultrasonic sensor handler
   }
   ```

## Future Enhancements

- Add UART/USB communication for telemetry
- Implement Kalman filter for IMU data fusion
- Add servo control for camera pan/tilt
- Implement PID controller for motor speed regulation
- Add SD card logging for sensor data
- Wireless communication (Bluetooth/WiFi)
