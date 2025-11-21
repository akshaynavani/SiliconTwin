#include "sim/mcu.h"

#include <string.h>

static void gpio_dispatch(void *ctx, uint8_t pin, bool level)
{
    mcu_t *mcu = (mcu_t *)ctx;
    if (mcu->gpio_isr) {
        hal_sim_bind(mcu);
        mcu->gpio_isr(pin, level);
    }
}

void mcu_init(mcu_t *mcu, sim_clock_t *clock, sim_rng_t *rng, uint32_t uart_baud,
              uint32_t pwm_freq_hz, can_bus_t *bus, uint8_t can_node,
              const mcu_firmware_t *fw)
{
    memset(mcu, 0, sizeof(*mcu));
    mcu->clock = clock;
    mcu->rng = rng;
    gpio_init(&mcu->gpio);
    uart_init(&mcu->uart, uart_baud, rng);
    adc_init(&mcu->adc, HAL_ADC_NUM_CHANNELS, HAL_ADC_BITS, rng);
    adc_set_full_scale(&mcu->adc, HAL_ADC_VOUT, HAL_ADC_VOUT_FULL_SCALE);
    adc_set_full_scale(&mcu->adc, HAL_ADC_IOUT, HAL_ADC_IOUT_FULL_SCALE);
    adc_set_full_scale(&mcu->adc, HAL_ADC_VIN, HAL_ADC_VIN_FULL_SCALE);
    adc_set_full_scale(&mcu->adc, HAL_ADC_TEMP, HAL_ADC_TEMP_FULL_SCALE);
    pwm_init(&mcu->pwm, pwm_freq_hz ? pwm_freq_hz : 100000u, 0.01, 0.95);
    wdt_init(&mcu->wdt);
    mcu->can_bus = bus;
    mcu->can_node = can_node;
    if (fw) {
        mcu->fw = *fw;
    }
    /* Erased flash reads as 0xFF. */
    memset(mcu->flash, 0xFF, sizeof(mcu->flash));
}

void mcu_power_on(mcu_t *mcu)
{
    /* Retained RAM holds garbage after power-on; firmware must validate it. */
    uint8_t *ram = (uint8_t *)mcu->retained_ram;
    for (size_t i = 0; i < MCU_RETAINED_RAM_SIZE; i++) {
        ram[i] = (uint8_t)(mcu->rng ? sim_rng_next(mcu->rng) : 0xA5);
    }
    mcu_reset(mcu, HAL_RESET_POWER_ON);
}

void mcu_reset(mcu_t *mcu, hal_reset_cause_t cause)
{
    sim_time_us now = mcu->clock->now;

    if (mcu->reset_count < MCU_RESET_LOG_SIZE) {
        mcu->reset_log[mcu->reset_count].timestamp_us = now;
        mcu->reset_log[mcu->reset_count].cause = cause;
    }
    mcu->reset_count++;
    mcu->reset_cause = cause;
    mcu->reset_pending = false;

    /* Peripherals return to reset state. */
    gpio_reset(&mcu->gpio);
    gpio_set_irq_handler(&mcu->gpio, gpio_dispatch, mcu);
    gpio_irq_enable(&mcu->gpio, true);
    uart_reset(&mcu->uart);
    pwm_reset(&mcu->pwm);
    wdt_reset(&mcu->wdt);
    if (mcu->can_bus) {
        can_node_reset(mcu->can_bus, mcu->can_node);
    }
    mcu->control_isr = NULL;
    sim_periodic_stop(&mcu->control_timer);
    mcu->gpio_isr = NULL;

    /* A reset un-sticks a hung super-loop (the hang was a firmware state). */
    mcu->hung = false;
    mcu->hang_until = 0;

    hal_sim_bind(mcu);
    if (mcu->fw.init) {
        mcu->fw.init();
    }
    sim_periodic_start(&mcu->main_timer, now, MCU_MAIN_TICK_US);
}

void mcu_request_reset(mcu_t *mcu, hal_reset_cause_t cause)
{
    mcu->reset_pending = true;
    mcu->pending_cause = cause;
}

void mcu_step(mcu_t *mcu)
{
    sim_time_us now = mcu->clock->now;
    hal_sim_bind(mcu);

    if (wdt_step(&mcu->wdt, now)) {
        mcu_reset(mcu, HAL_RESET_WATCHDOG);
        return;
    }

    if (mcu->control_isr && sim_periodic_due(&mcu->control_timer, now)) {
        mcu->control_isr_count++;
        mcu->control_isr();
    }

    if (mcu->hung && mcu->hang_until != 0 && now >= mcu->hang_until) {
        mcu->hung = false;
    }

    if (sim_periodic_due(&mcu->main_timer, now) && !mcu->hung && mcu->fw.main_tick) {
        mcu->main_tick_count++;
        mcu->fw.main_tick();
    }

    if (mcu->reset_pending) {
        mcu_reset(mcu, mcu->pending_cause);
    }
}

void mcu_hang(mcu_t *mcu, sim_time_us duration_us)
{
    mcu->hung = true;
    mcu->hang_until = duration_us ? mcu->clock->now + duration_us : 0;
}

void mcu_unhang(mcu_t *mcu)
{
    mcu->hung = false;
    mcu->hang_until = 0;
}

bool mcu_flash_program(mcu_t *mcu, uint32_t offset, const void *data, size_t len)
{
    if ((size_t)offset + len > MCU_FLASH_SIZE) {
        return false;
    }
    memcpy(&mcu->flash[offset], data, len);
    return true;
}

const char *mcu_reset_cause_name(hal_reset_cause_t cause)
{
    switch (cause) {
    case HAL_RESET_POWER_ON: return "power_on";
    case HAL_RESET_WATCHDOG: return "watchdog";
    case HAL_RESET_SOFTWARE: return "software";
    case HAL_RESET_BROWNOUT: return "brownout";
    }
    return "unknown";
}
