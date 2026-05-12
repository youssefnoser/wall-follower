#include "esp_ring.h"
#include "stm32f1xx_hal.h"  /* __disable_irq / __enable_irq */

#define MASK (ESP_RING_SIZE - 1u)

static uint8_t           ring[ESP_RING_SIZE];
static volatile uint16_t head     = 0;  /* write ptr — ISR side */
static          uint16_t cmd_tail = 0;  /* AT-command read start */
static          uint16_t urc_tail = 0;  /* URC / +IPD read start */

void esp_ring_init(void)
{
    head = cmd_tail = urc_tail = 0;
}

/* ISR side */

void esp_ring_push(uint8_t byte)
{
    /* If the ring is full, drop the oldest unread byte by advancing both
     * tails so the buffer never stalls. */
    if ((uint16_t)(head - urc_tail) >= ESP_RING_SIZE) {
        urc_tail++;
        if ((uint16_t)(head - cmd_tail) >= ESP_RING_SIZE)
            cmd_tail++;
    }
    ring[head & MASK] = byte;
    head++;
}

/* AT-command cursor */

void esp_ring_cmd_mark(void)
{
    /* Capture head atomically (16-bit read is atomic on Cortex-M3). */
    cmd_tail = head;
}

uint16_t esp_ring_cmd_drain(uint8_t *dst, uint16_t max)
{
    uint16_t avail = (uint16_t)(head - cmd_tail);
    if (avail > max) avail = max;
    for (uint16_t i = 0; i < avail; i++)
        dst[i] = ring[(cmd_tail + i) & MASK];
    cmd_tail += avail;
    return avail;
}

/* URC / +IPD cursor */

uint16_t esp_ring_urc_available(void)
{
    return (uint16_t)(head - urc_tail);
}

uint16_t esp_ring_urc_peek(uint8_t *dst, uint16_t max)
{
    uint16_t avail = (uint16_t)(head - urc_tail);
    if (avail > max) avail = max;
    for (uint16_t i = 0; i < avail; i++)
        dst[i] = ring[(urc_tail + i) & MASK];
    return avail;
}

void esp_ring_urc_consume(uint16_t n)
{
    urc_tail += n;
}
