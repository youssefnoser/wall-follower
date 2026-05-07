#ifndef ESP_RING_H
#define ESP_RING_H

/*
 * esp_ring — lock-free UART RX ring with two independent read cursors.
 *
 * Problem the existing driver has
 * --------------------------------
 * Wifi_RxClear() wipes the shared RxBuffer before every AT command, which
 * destroys any +IPD / URC bytes that arrived between commands.  The fix is
 * to keep raw bytes in a ring that is NEVER cleared, and give AT-command
 * responses and async URC/+IPD traffic separate read pointers.
 *
 * Two cursors
 * -----------
 *  cmd_tail  – moved by Wifi_RxClear() to skip stale bytes; advanced by
 *              esp_ring_cmd_drain() which feeds Wifi_WaitForString().
 *
 *  urc_tail  – advanced by esp_ring_urc_consume(), used by poll_async_events()
 *              in the application.  Never touched by AT-command code.
 *
 * Both cursors wrap at ESP_RING_SIZE (power of 2, so masking works).
 * uint16_t indices overflow naturally and the arithmetic stays correct as long
 * as no more than ESP_RING_SIZE unread bytes accumulate.
 *
 * ISR safety
 * ----------
 * esp_ring_push() is called from UART RX ISR (single-byte IT mode).
 * `head` is declared volatile uint16_t; Cortex-M3 load/store of a 16-bit
 * value is atomic, so readers only need to read `head` once into a local.
 */

#include <stdint.h>

#define ESP_RING_SIZE 512u   /* must be a power of 2 */

void     esp_ring_init(void);

/* Called from UART RX ISR — push one received byte. */
void     esp_ring_push(uint8_t byte);

/* ---- AT-command cursor ------------------------------------------------ */

/* Mark the ring head as the start of a new AT command's response window.
 * Call this instead of memset-ing the receive buffer. */
void     esp_ring_cmd_mark(void);

/* Drain up to `max` bytes that arrived after the last cmd_mark into `dst`.
 * Returns the number of bytes copied; advances the cmd cursor. */
uint16_t esp_ring_cmd_drain(uint8_t *dst, uint16_t max);

/* ---- URC / +IPD cursor ----------------------------------------------- */

/* Bytes available for the async (URC/+IPD) consumer. */
uint16_t esp_ring_urc_available(void);

/* Copy up to `max` bytes into `dst` without consuming.  Returns count. */
uint16_t esp_ring_urc_peek(uint8_t *dst, uint16_t max);

/* Mark the first `n` bytes as consumed by the URC consumer. */
void     esp_ring_urc_consume(uint16_t n);

#endif /* ESP_RING_H */
