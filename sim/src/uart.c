#include "sim/uart.h"

#include <string.h>

void byte_fifo_init(byte_fifo_t *f, uint8_t *storage, uint16_t capacity)
{
    f->buf = storage;
    f->capacity = capacity;
    byte_fifo_clear(f);
}

void byte_fifo_clear(byte_fifo_t *f)
{
    f->head = 0;
    f->tail = 0;
    f->count = 0;
}

bool byte_fifo_push(byte_fifo_t *f, uint8_t b)
{
    if (f->count >= f->capacity) {
        return false;
    }
    f->buf[f->head] = b;
    f->head = (uint16_t)((f->head + 1u) % f->capacity);
    f->count++;
    return true;
}

bool byte_fifo_pop(byte_fifo_t *f, uint8_t *b)
{
    if (f->count == 0) {
        return false;
    }
    *b = f->buf[f->tail];
    f->tail = (uint16_t)((f->tail + 1u) % f->capacity);
    f->count--;
    return true;
}

void uart_init(uart_t *u, uint32_t baud, sim_rng_t *rng)
{
    memset(u, 0, sizeof(*u));
    u->baud = baud ? baud : 115200;
    /* 10 bits per frame, rounded up so we never transmit faster than the line allows. */
    u->byte_time_us = (sim_time_us)((10ULL * 1000000ULL + u->baud - 1) / u->baud);
    u->rng = rng;
    byte_fifo_init(&u->tx_fifo, u->tx_storage, UART_HW_FIFO_SIZE);
    byte_fifo_init(&u->rx_fifo, u->rx_storage, UART_HW_FIFO_SIZE);
    byte_fifo_init(&u->wire_in, u->wire_storage, UART_WIRE_QUEUE_SIZE);
    u->capture[0] = '\0';
}

void uart_reset(uart_t *u)
{
    byte_fifo_clear(&u->tx_fifo);
    byte_fifo_clear(&u->rx_fifo);
    u->enabled = false;
    u->overrun_flag = false;
    u->framing_error_flag = false;
    u->tx_busy_until = 0;
}

static void capture_byte(uart_t *u, uint8_t b)
{
    if (u->capture_len >= UART_CAPTURE_SIZE) {
        u->stats.capture_dropped++;
        return;
    }
    u->capture[u->capture_len++] = (char)b;
    u->capture[u->capture_len] = '\0';
}

void uart_step(uart_t *u, sim_time_us now)
{
    /* Transmit: shift out at most one byte per byte-time. */
    if (u->enabled && now >= u->tx_busy_until) {
        uint8_t b;
        if (byte_fifo_pop(&u->tx_fifo, &b)) {
            capture_byte(u, b);
            u->stats.tx_bytes++;
            u->tx_busy_until = now + u->byte_time_us;
        }
    }

    /* Receive: the wire delivers one byte per byte-time regardless of whether
     * the MCU is listening, which is what makes overruns possible. */
    if (u->wire_in.count > 0 && now >= u->rx_next_at) {
        uint8_t b;
        byte_fifo_pop(&u->wire_in, &b);
        u->rx_next_at = now + u->byte_time_us;
        if (!u->enabled) {
            return; /* receiver off: byte is lost on the floor */
        }
        if (u->rng && sim_rng_chance(u->rng, u->noise_rate)) {
            /* Line noise: flip a random bit and flag a framing error. */
            b ^= (uint8_t)(1u << (sim_rng_next(u->rng) % 8u));
            u->framing_error_flag = true;
            u->stats.framing_errors++;
        }
        if (byte_fifo_push(&u->rx_fifo, b)) {
            u->stats.rx_bytes++;
        } else {
            u->overrun_flag = true;
            u->stats.overruns++;
        }
    }
}

void uart_enable(uart_t *u, bool enabled)
{
    u->enabled = enabled;
}

size_t uart_write(uart_t *u, const uint8_t *data, size_t len)
{
    size_t n = 0;
    while (n < len && byte_fifo_push(&u->tx_fifo, data[n])) {
        n++;
    }
    return n;
}

size_t uart_read(uart_t *u, uint8_t *data, size_t max)
{
    size_t n = 0;
    while (n < max && byte_fifo_pop(&u->rx_fifo, &data[n])) {
        n++;
    }
    return n;
}

size_t uart_rx_available(const uart_t *u)
{
    return u->rx_fifo.count;
}

size_t uart_tx_free(const uart_t *u)
{
    return (size_t)(u->tx_fifo.capacity - u->tx_fifo.count);
}

bool uart_take_overrun(uart_t *u)
{
    bool f = u->overrun_flag;
    u->overrun_flag = false;
    return f;
}

bool uart_take_framing_error(uart_t *u)
{
    bool f = u->framing_error_flag;
    u->framing_error_flag = false;
    return f;
}

size_t uart_host_send(uart_t *u, const char *data, size_t len)
{
    size_t n = 0;
    while (n < len && byte_fifo_push(&u->wire_in, (uint8_t)data[n])) {
        n++;
    }
    return n;
}

const char *uart_host_output(const uart_t *u, size_t *len)
{
    if (len) {
        *len = u->capture_len;
    }
    return u->capture;
}

void uart_host_clear_output(uart_t *u)
{
    u->capture_len = 0;
    u->capture[0] = '\0';
}

void uart_set_noise(uart_t *u, double probability)
{
    u->noise_rate = probability;
}
