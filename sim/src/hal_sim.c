/*
 * HAL implementation backed by the simulated MCU.
 *
 * The firmware is single-instance (it keeps its state in static storage, as
 * real firmware does), so the HAL is bound to one mcu_t at a time. The MCU
 * model re-binds before every call into firmware.
 */
#include <string.h>

#include "fw/hal.h"
#include "sim/mcu.h"

static mcu_t *g_mcu;

void hal_sim_bind(mcu_t *mcu)
{
    g_mcu = mcu;
}

mcu_t *hal_sim_bound(void)
{
    return g_mcu;
}

/* ------------------------------------------------------------------ time */
uint32_t hal_millis(void)
{
    return g_mcu ? (uint32_t)(g_mcu->clock->now / SIM_US_PER_MS) : 0u;
}

uint64_t hal_micros(void)
{
    return g_mcu ? g_mcu->clock->now : 0u;
}

/* ------------------------------------------------------------------ GPIO */
void hal_gpio_config_output(uint8_t pin, bool initial)
{
    gpio_configure(&g_mcu->gpio, pin, GPIO_DIR_OUTPUT, GPIO_PULL_NONE);
    gpio_write(&g_mcu->gpio, pin, initial);
}

void hal_gpio_config_input(uint8_t pin, bool pull_up, bool pull_down)
{
    gpio_pull_t pull = GPIO_PULL_NONE;
    if (pull_up) {
        pull = GPIO_PULL_UP;
    } else if (pull_down) {
        pull = GPIO_PULL_DOWN;
    }
    gpio_configure(&g_mcu->gpio, pin, GPIO_DIR_INPUT, pull);
}

void hal_gpio_write(uint8_t pin, bool level)
{
    gpio_write(&g_mcu->gpio, pin, level);
}

void hal_gpio_toggle(uint8_t pin)
{
    gpio_toggle(&g_mcu->gpio, pin);
}

bool hal_gpio_read(uint8_t pin)
{
    return gpio_read(&g_mcu->gpio, pin);
}

void hal_gpio_enable_irq(uint8_t pin, hal_edge_t edge)
{
    gpio_set_edge(&g_mcu->gpio, pin, (gpio_edge_t)edge);
}

void hal_gpio_set_isr(hal_gpio_isr_t isr)
{
    g_mcu->gpio_isr = isr;
}

/* ------------------------------------------------------------- ADC / PWM */
uint16_t hal_adc_read(hal_adc_channel_t ch)
{
    return adc_convert(&g_mcu->adc, (uint8_t)ch);
}

void hal_pwm_enable(bool enabled)
{
    pwm_enable(&g_mcu->pwm, enabled);
}

void hal_pwm_set_duty(float duty)
{
    pwm_set_duty(&g_mcu->pwm, (double)duty);
}

void hal_set_control_isr(hal_isr_t isr, uint32_t rate_hz)
{
    g_mcu->control_isr = isr;
    if (isr && rate_hz > 0) {
        sim_periodic_start(&g_mcu->control_timer, g_mcu->clock->now, 1000000u / rate_hz);
    } else {
        sim_periodic_stop(&g_mcu->control_timer);
    }
}

/* ------------------------------------------------------------------ UART */
void hal_uart_init(void)
{
    uart_enable(&g_mcu->uart, true);
}

size_t hal_uart_write(const void *data, size_t len)
{
    return uart_write(&g_mcu->uart, (const uint8_t *)data, len);
}

size_t hal_uart_read(void *data, size_t max)
{
    return uart_read(&g_mcu->uart, (uint8_t *)data, max);
}

bool hal_uart_take_error(void)
{
    bool overrun = uart_take_overrun(&g_mcu->uart);
    bool framing = uart_take_framing_error(&g_mcu->uart);
    return overrun || framing;
}

/* ------------------------------------------------------------------- CAN */
void hal_can_init(uint32_t bitrate)
{
    (void)bitrate; /* the bus bitrate is a property of the bench wiring */
    if (g_mcu->can_bus) {
        can_node_enable(g_mcu->can_bus, g_mcu->can_node, true);
    }
}

bool hal_can_set_filter(uint8_t index, uint16_t id, uint16_t mask)
{
    if (!g_mcu->can_bus) {
        return false;
    }
    return can_node_set_filter(g_mcu->can_bus, g_mcu->can_node, index, id, mask);
}

bool hal_can_send(const hal_can_frame_t *frame)
{
    if (!g_mcu->can_bus) {
        return false;
    }
    can_frame_t f;
    f.id = frame->id;
    f.dlc = frame->dlc;
    memcpy(f.data, frame->data, sizeof(f.data));
    return can_node_send(g_mcu->can_bus, g_mcu->can_node, &f);
}

bool hal_can_receive(hal_can_frame_t *frame)
{
    if (!g_mcu->can_bus) {
        return false;
    }
    can_frame_t f;
    if (!can_node_receive(g_mcu->can_bus, g_mcu->can_node, &f)) {
        return false;
    }
    frame->id = f.id;
    frame->dlc = f.dlc;
    memcpy(frame->data, f.data, sizeof(frame->data));
    return true;
}

hal_can_state_t hal_can_state(void)
{
    if (!g_mcu->can_bus) {
        return HAL_CAN_BUS_OFF;
    }
    return (hal_can_state_t)can_node_state(g_mcu->can_bus, g_mcu->can_node);
}

void hal_can_recover(void)
{
    if (g_mcu->can_bus) {
        can_node_request_recovery(g_mcu->can_bus, g_mcu->can_node, g_mcu->clock->now);
    }
}

/* ------------------------------------------------------ watchdog / reset */
void hal_wdt_start(uint32_t timeout_ms)
{
    wdt_start(&g_mcu->wdt, g_mcu->clock->now, SIM_MS(timeout_ms));
}

void hal_wdt_kick(void)
{
    wdt_kick(&g_mcu->wdt, g_mcu->clock->now);
}

hal_reset_cause_t hal_reset_cause(void)
{
    return g_mcu->reset_cause;
}

void hal_system_reset(void)
{
    mcu_request_reset(g_mcu, HAL_RESET_SOFTWARE);
}

/* -------------------------------------------------------------- memories */
void *hal_retained_ram(size_t *size)
{
    if (size) {
        *size = MCU_RETAINED_RAM_SIZE;
    }
    return g_mcu->retained_ram;
}

size_t hal_flash_read(uint32_t offset, void *dst, size_t len)
{
    if (offset >= MCU_FLASH_SIZE) {
        return 0;
    }
    size_t avail = MCU_FLASH_SIZE - offset;
    size_t n = len < avail ? len : avail;
    memcpy(dst, &g_mcu->flash[offset], n);
    return n;
}
