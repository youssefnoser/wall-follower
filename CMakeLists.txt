cmake_minimum_required(VERSION 3.20)

# Must be before project() — toolchain file is passed via CMakePresets.json
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

project(STM32_Motor_Control C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# ── MCU identity ──────────────────────────────────────────────────────────────
set(STM32_FAMILY STM32F1xx)
set(STM32_MODEL  STM32F103C8)

# ── Cortex-M3: NO -mfpu / -mfloat-abi (hardware FPU does not exist) ──────────
set(MCU_FLAGS "-mcpu=cortex-m3 -mthumb")

set(CMAKE_C_FLAGS
    "${MCU_FLAGS} -Wall -Wextra -Wno-unused-parameter -Og -g3 -fdata-sections -ffunction-sections")
set(CMAKE_ASM_FLAGS "${MCU_FLAGS} -x assembler-with-cpp")

# ── Post-build tools ──────────────────────────────────────────────────────────
find_program(CMAKE_OBJCOPY arm-none-eabi-objcopy REQUIRED)
find_program(CMAKE_OBJDUMP arm-none-eabi-objdump)
find_program(CMAKE_SIZE    arm-none-eabi-size)

# ── Include paths ─────────────────────────────────────────────────────────────
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/STM32F1xx_HAL_Driver/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/Device/ST/STM32F1xx/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/CMSIS/inc
)

# ── HAL define: tells stm32f1xx.h which device variant to use ─────────────────
# STM32F103x8 = 64 KB Flash  |  STM32F103xB = 128 KB Flash
# Change to STM32F103xB if you use the 128 KB variant
add_compile_definitions(STM32F103xB USE_HAL_DRIVER)

# ── HAL sources — add/remove as your project needs ────────────────────────────
set(HAL_SOURCES
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_cortex.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_rcc.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_rcc_ex.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_gpio.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_dma.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_tim.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_tim_ex.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_i2c.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_adc.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_adc_ex.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_pwr.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_flash.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_flash_ex.c
    drivers/STM32F1xx_HAL_Driver/src/stm32f1xx_hal_uart.c

    # CMSIS system init
    drivers/CMSIS/Device/ST/STM32F1xx/source/Templates/system_stm32f1xx.c
)

# ── Startup file (GCC variant for STM32F103x8) ────────────────────────────────
set(STARTUP_FILE
    drivers/CMSIS/Device/ST/STM32F1xx/source/Templates/gcc/startup_stm32f103xb.s)
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${STARTUP_FILE}")
    message(FATAL_ERROR
        "Startup file not found: ${STARTUP_FILE}\n"
        "Clone the STM32F1xx CMSIS device pack:\n"
        "  git clone --depth 1 https://github.com/STMicroelectronics/cmsis_device_f1.git\n"
        "Then copy Include/ → drivers/CMSIS/Device/ST/STM32F1xx/inc/\n"
        "     and Source/  → drivers/CMSIS/Device/ST/STM32F1xx/source/")
endif()

# ── Linker script ─────────────────────────────────────────────────────────────
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8Tx_FLASH.ld")
if(NOT EXISTS "${LINKER_SCRIPT}")
    message(FATAL_ERROR
        "Linker script not found: ${LINKER_SCRIPT}\n"
        "It should have been generated alongside this CMakeLists.")
endif()

# ── Executable ────────────────────────────────────────────────────────────────
set(SOURCES
    src/main.c
    src/l298n.c
    src/hc_sr04.c
    src/ir_sensor.c
    src/mpu6050.c
    src/ESP8266.c
    src/dwt_stm32_delay.c
    src/wall_follower_fsm.c
    src/wifi_control.c
    ${HAL_SOURCES}
    ${STARTUP_FILE}
)

add_executable(${PROJECT_NAME}.elf ${SOURCES})

target_link_options(${PROJECT_NAME}.elf PRIVATE
    "SHELL:${MCU_FLAGS}"       # SHELL: prevents CMake quoting the flags as one token
    -Wl,--gc-sections
    -Wl,--print-memory-usage
    -Wl,-Map=${PROJECT_NAME}.map
    --specs=nano.specs
    --specs=rdimon.specs
    -u_printf_float
    -T${LINKER_SCRIPT}
)

# ── Post-build: hex + bin + size ──────────────────────────────────────────────
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating .hex / .bin"
)

if(CMAKE_SIZE)
    add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Memory usage"
    )
endif()

# ── Flash target (openocd) ────────────────────────────────────────────────────
find_program(OPENOCD openocd)
if(OPENOCD)
    add_custom_target(flash
        COMMAND ${OPENOCD}
            -f interface/stlink-v2.cfg
            -c "set CPUTAPID 0x2ba01477"
            -f target/stm32f1x.cfg
            -c "program ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.elf verify reset exit"
        DEPENDS ${PROJECT_NAME}.elf
        COMMENT "Flashing via OpenOCD"
    )
endif()

message(STATUS "Compiler : ${CMAKE_C_COMPILER}")
message(STATUS "MCU      : ${STM32_MODEL}")
message(STATUS "C Flags  : ${CMAKE_C_FLAGS}")