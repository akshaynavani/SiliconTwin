#include "sim/gpio.h"

#include <string.h>

static bool valid_pin(uint8_t pin)
{
    return pin < GPIO_NUM_PINS;
}

static bool resolve_level(const gpio_pin_t *p)
{
    if (p->stuck) {
        return p->stuck_level;
    }
    if (p->dir == GPIO_DIR_OUTPUT) {
        return p->out_level;
    }
    if (p->ext_driven) {
        return p->ext_level;
    }
    /* Floating inputs read low unless pulled up. */
    return p->pull == GPIO_PULL_UP;
}

static void update_pin(gpio_port_t *port, uint8_t pin)
{
    gpio_pin_t *p = &port->pins[pin];
    bool old = p->level;
    bool now = resolve_level(p);
    if (old == now) {
        return;
    }
    p->level = now;
    p->toggle_count++;

    if (p->dir != GPIO_DIR_INPUT) {
        return;
    }
    bool rising = !old && now;
    bool falling = old && !now;
    bool fire = (rising && (p->edge & GPIO_EDGE_RISING)) || (falling && (p->edge & GPIO_EDGE_FALLING));
    if (fire) {
        p->edge_count++;
        if (port->irq_enabled && port->irq_cb) {
            port->irq_cb(port->irq_ctx, pin, now);
        }
    }
}

void gpio_init(gpio_port_t *port)
{
    memset(port, 0, sizeof(*port));
}

void gpio_reset(gpio_port_t *port)
{
    for (uint8_t i = 0; i < GPIO_NUM_PINS; i++) {
        gpio_pin_t *p = &port->pins[i];
        p->dir = GPIO_DIR_INPUT;
        p->pull = GPIO_PULL_NONE;
        p->edge = GPIO_EDGE_NONE;
        p->out_level = false;
        p->level = resolve_level(p);
    }
    port->irq_cb = NULL;
    port->irq_ctx = NULL;
    port->irq_enabled = false;
}

bool gpio_configure(gpio_port_t *port, uint8_t pin, gpio_dir_t dir, gpio_pull_t pull)
{
    if (!valid_pin(pin)) {
        return false;
    }
    port->pins[pin].dir = dir;
    port->pins[pin].pull = pull;
    update_pin(port, pin);
    return true;
}

bool gpio_set_edge(gpio_port_t *port, uint8_t pin, gpio_edge_t edge)
{
    if (!valid_pin(pin)) {
        return false;
    }
    port->pins[pin].edge = edge;
    return true;
}

bool gpio_write(gpio_port_t *port, uint8_t pin, bool level)
{
    if (!valid_pin(pin) || port->pins[pin].dir != GPIO_DIR_OUTPUT) {
        return false;
    }
    port->pins[pin].out_level = level;
    update_pin(port, pin);
    return true;
}

bool gpio_toggle(gpio_port_t *port, uint8_t pin)
{
    if (!valid_pin(pin)) {
        return false;
    }
    return gpio_write(port, pin, !port->pins[pin].out_level);
}

bool gpio_read(const gpio_port_t *port, uint8_t pin)
{
    if (!valid_pin(pin)) {
        return false;
    }
    return port->pins[pin].level;
}

void gpio_set_irq_handler(gpio_port_t *port, gpio_irq_cb cb, void *ctx)
{
    port->irq_cb = cb;
    port->irq_ctx = ctx;
}

void gpio_irq_enable(gpio_port_t *port, bool enabled)
{
    port->irq_enabled = enabled;
}

bool gpio_drive_external(gpio_port_t *port, uint8_t pin, bool level)
{
    if (!valid_pin(pin)) {
        return false;
    }
    port->pins[pin].ext_driven = true;
    port->pins[pin].ext_level = level;
    update_pin(port, pin);
    return true;
}

bool gpio_release_external(gpio_port_t *port, uint8_t pin)
{
    if (!valid_pin(pin)) {
        return false;
    }
    port->pins[pin].ext_driven = false;
    update_pin(port, pin);
    return true;
}

bool gpio_inject_stuck(gpio_port_t *port, uint8_t pin, bool level)
{
    if (!valid_pin(pin)) {
        return false;
    }
    port->pins[pin].stuck = true;
    port->pins[pin].stuck_level = level;
    update_pin(port, pin);
    return true;
}

bool gpio_clear_stuck(gpio_port_t *port, uint8_t pin)
{
    if (!valid_pin(pin)) {
        return false;
    }
    port->pins[pin].stuck = false;
    update_pin(port, pin);
    return true;
}
