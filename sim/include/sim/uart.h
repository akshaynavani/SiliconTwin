/*
 * UART emulation with byte-accurate timing.
 *
 * One byte occupies 10 bit times on the wire (start + 8 data + stop), so at
 * 115200 baud a byte takes ~87 us. The MCU side sees small hardware FIFOs; if
 * firmware does not drain the RX FIFO fast enough the overrun flag is set and
 * bytes are dropped, just like a real peripheral. The host side has a larger
 * "wire" queue for injected input and a capture buffer for transmitted output.
 */
#ifndef SIM_UART_H
#define SIM_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim/clock.h"
#include "sim/rng.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_HW_FIFO_SIZE 32
#define UART_WIRE_QUEUE_SIZE 2048
#define UART_CAPTURE_SIZE 16384

typedef struct {
    uint8_t *buf;
    uint16_t capacity;
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} byte_fifo_t;

typedef struct {
    uint32_t tx_bytes;
    uint32_t rx_bytes;
    uint32_t overruns;
    uint32_t framing_errors;
    uint32_t capture_dropped;
} uart_stats_t;

typedef struct {
    uint32_t baud;
    sim_time_us byte_time_us;
    bool enabled;

    uint8_t tx_storage[UART_HW_FIFO_SIZE];
    uint8_t rx_storage[UART_HW_FIFO_SIZE];
    uint8_t wire_storage[UART_WIRE_QUEUE_SIZE];
    byte_fifo_t tx_fifo;
    byte_fifo_t rx_fifo;
    byte_fifo_t wire_in; /* host -> MCU bytes not yet clocked in */

    sim_time_us tx_busy_until;
    sim_time_us rx_next_at;

    char capture[UART_CAPTURE_SIZE + 1]; /* MCU -> host, NUL terminated */
    size_t capture_len;

    bool overrun_flag;
    bool framing_error_flag;
    double noise_rate; /* probability a received byte is corrupted */
    sim_rng_t *rng;

    uart_stats_t stats;
} uart_t;

void byte_fifo_init(byte_fifo_t *f, uint8_t *storage, uint16_t capacity);
bool byte_fifo_push(byte_fifo_t *f, uint8_t b);
bool byte_fifo_pop(byte_fifo_t *f, uint8_t *b);
void byte_fifo_clear(byte_fifo_t *f);

void uart_init(uart_t *u, uint32_t baud, sim_rng_t *rng);
/* MCU reset: clears FIFOs and flags but keeps host capture and wire queue. */
void uart_reset(uart_t *u);
void uart_step(uart_t *u, sim_time_us now);

/* --- MCU side --- */
void uart_enable(uart_t *u, bool enabled);
size_t uart_write(uart_t *u, const uint8_t *data, size_t len);
size_t uart_read(uart_t *u, uint8_t *data, size_t max);
size_t uart_rx_available(const uart_t *u);
size_t uart_tx_free(const uart_t *u);
/* Returns and clears the sticky overrun / framing error flags. */
bool uart_take_overrun(uart_t *u);
bool uart_take_framing_error(uart_t *u);

/* --- Host side --- */
size_t uart_host_send(uart_t *u, const char *data, size_t len);
const char *uart_host_output(const uart_t *u, size_t *len);
void uart_host_clear_output(uart_t *u);
void uart_set_noise(uart_t *u, double probability);

#ifdef __cplusplus
}
#endif

#endif /* SIM_UART_H */
