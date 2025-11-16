/*
 * GPIO port emulation.
 *
 * Each pin resolves its logic level from (in priority order): an injected
 * stuck-at fault, the MCU's output latch, an external driver (test bench /
 * scenario), and finally the configured pull resistor. Input pins configured
 * for edge interrupts invoke the registered callback synchronously, which is
 * how the simulated MCU dispatches GPIO ISRs into firmware.
 */
#ifndef SIM_GPIO_H
#define SIM_GPIO_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_NUM_PINS 16

typedef enum { GPIO_DIR_INPUT = 0, GPIO_DIR_OUTPUT = 1 } gpio_dir_t;
typedef enum { GPIO_PULL_NONE = 0, GPIO_PULL_UP = 1, GPIO_PULL_DOWN = 2 } gpio_pull_t;
typedef enum {
    GPIO_EDGE_NONE = 0,
    GPIO_EDGE_RISING = 1,
    GPIO_EDGE_FALLING = 2,
    GPIO_EDGE_BOTH = 3
} gpio_edge_t;

typedef void (*gpio_irq_cb)(void *ctx, uint8_t pin, bool level);

typedef struct {
    gpio_dir_t dir;
    gpio_pull_t pull;
    gpio_edge_t edge;
    bool out_level;   /* MCU output latch */
    bool ext_driven;  /* driven by the outside world */
    bool ext_level;
    bool stuck;       /* fault injection: stuck-at */
    bool stuck_level;
    bool level;       /* resolved level */
    uint32_t edge_count;
    uint32_t toggle_count; /* level changes, used to verify output activity */
} gpio_pin_t;

typedef struct {
    gpio_pin_t pins[GPIO_NUM_PINS];
    gpio_irq_cb irq_cb;
    void *irq_ctx;
    bool irq_enabled;
} gpio_port_t;

void gpio_init(gpio_port_t *port);
/* Full reset of MCU-side configuration; external drivers and faults persist. */
void gpio_reset(gpio_port_t *port);

/* --- MCU (firmware-facing) side --- */
bool gpio_configure(gpio_port_t *port, uint8_t pin, gpio_dir_t dir, gpio_pull_t pull);
bool gpio_set_edge(gpio_port_t *port, uint8_t pin, gpio_edge_t edge);
bool gpio_write(gpio_port_t *port, uint8_t pin, bool level);
bool gpio_toggle(gpio_port_t *port, uint8_t pin);
bool gpio_read(const gpio_port_t *port, uint8_t pin);
void gpio_set_irq_handler(gpio_port_t *port, gpio_irq_cb cb, void *ctx);
void gpio_irq_enable(gpio_port_t *port, bool enabled);

/* --- External (test bench) side --- */
bool gpio_drive_external(gpio_port_t *port, uint8_t pin, bool level);
bool gpio_release_external(gpio_port_t *port, uint8_t pin);
bool gpio_inject_stuck(gpio_port_t *port, uint8_t pin, bool level);
bool gpio_clear_stuck(gpio_port_t *port, uint8_t pin);

#ifdef __cplusplus
}
#endif

#endif /* SIM_GPIO_H */
