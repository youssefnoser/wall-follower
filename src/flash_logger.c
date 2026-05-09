#include "flash_logger.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>

#define FLASH_LOG_START_ADDR 0x0800F000
#define FLASH_LOG_END_ADDR   0x0800FFFF // 4KB, 4 pages (60, 61, 62, 63)

static uint32_t current_log_addr = FLASH_LOG_START_ADDR;
static uint8_t byte_buf = 0;
static bool has_byte = false;

void flash_logger_init(void) {
    // Find the first unwritten halfword (0xFFFF)
    current_log_addr = FLASH_LOG_START_ADDR;
    while (current_log_addr < FLASH_LOG_END_ADDR) {
        if (*(__IO uint16_t*)current_log_addr == 0xFFFF) {
            break;
        }
        current_log_addr += 2;
    }
}

void flash_logger_clear(void) {
    HAL_FLASH_Unlock();
    
    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.PageAddress = FLASH_LOG_START_ADDR;
    erase_init.NbPages = 4; // 4 KB = 4 pages
    
    uint32_t page_error = 0;
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    
    HAL_FLASH_Lock();
    
    current_log_addr = FLASH_LOG_START_ADDR;
    has_byte = false;
}

void flash_logger_write(const char* data, int len) {
    HAL_FLASH_Unlock();
    for (int i = 0; i < len; i++) {
        if (current_log_addr >= FLASH_LOG_END_ADDR - 2) {
            break;
        }
        
        if (!has_byte) {
            byte_buf = data[i];
            has_byte = true;
        } else {
            uint16_t hw = ((uint16_t)data[i] << 8) | byte_buf;
            HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, current_log_addr, hw);
            current_log_addr += 2;
            has_byte = false;
        }
    }
    HAL_FLASH_Lock();
}
