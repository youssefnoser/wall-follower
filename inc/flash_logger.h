#ifndef FLASH_LOGGER_H
#define FLASH_LOGGER_H

#include <stdint.h>

void flash_logger_init(void);
void flash_logger_clear(void);
void flash_logger_write(const char* data, int len);

#endif // FLASH_LOGGER_H
