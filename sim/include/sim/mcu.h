/*
 * Simulated microcontroller.
 *
 * Aggregates the on-chip peripherals, the interrupt/tick scheduling and the
 * reset controller. Firmware is plugged in through two entry points:
 *   init()      - called after every reset (the "main()" prologue)
 *   main_tick() - one iteration of the super-loop, scheduled at 1 kHz
 * Interrupt handlers are registered by the firmware itself through the HAL.
 */
#ifndef SIM_MCU_H
#define SIM_MCU_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/hal.h"
#include "sim/adc.h"
#include "sim/can.h"
#include "sim/clock.h"
#include "sim/gpio.h"
#include "sim/pwm.h"
#include "sim/rng.h"
#include "sim/uart.h"
#include "sim/watchdog.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCU_RETAINED_RAM_SIZE 256
#define MCU_FLASH_SIZE 1024
#define MCU_RESET_LOG_SIZE 64
#define MCU_MAIN_TICK_US 1000u

typedef struct {
    void (*init)(void);
    void (*main_tick)(void);
} mcu_firmware_t;

typedef struct {
    sim_time_us timestamp_us;
    hal_reset_cause_t cause;
} mcu_reset_record_t;

typedef struct mcu {
    sim_clock_t *clock;
    sim_rng_t *rng;

    gpio_port_t gpio;
    uart_t uart;
    adc_t adc;
    pwm_t pwm;
    watchdog_t wdt;
    can_bus_t *can_bus;
    uint8_t can_node;

    /* Word-aligned so firmware can overlay structs on it. */
    uint64_t retained_ram[MCU_RETAINED_RAM_SIZE / sizeof(uint64_t)];
    uint8_t flash[MCU_FLASH_SIZE];

    mcu_firmware_t fw;
    hal_isr_t control_isr;
    sim_periodic_t control_timer;
    hal_gpio_isr_t gpio_isr;
    sim_periodic_t main_timer;

    hal_reset_cause_t reset_cause;
    mcu_reset_record_t reset_log[MCU_RESET_LOG_SIZE];
    uint32_t reset_count;           /* all resets including power-on */
    bool reset_pending;             /* software reset requested from firmware */
    hal_reset_cause_t pending_cause;

    /* Fault injection: the super-loop stops making progress (ISRs still run). */
    bool hung;
    sim_time_us hang_until;         /* 0 with hung=true means forever */

    uint64_t control_isr_count;
    uint64_t main_tick_count;
} mcu_t;

void mcu_init(mcu_t *mcu, sim_clock_t *clock, sim_rng_t *rng, uint32_t uart_baud,
              uint32_t pwm_freq_hz, can_bus_t *bus, uint8_t can_node,
              const mcu_firmware_t *fw);
void mcu_power_on(mcu_t *mcu);
void mcu_reset(mcu_t *mcu, hal_reset_cause_t cause);
/* Requests a reset that is performed after the current firmware call returns. */
void mcu_request_reset(mcu_t *mcu, hal_reset_cause_t cause);
void mcu_step(mcu_t *mcu);

void mcu_hang(mcu_t *mcu, sim_time_us duration_us);
void mcu_unhang(mcu_t *mcu);
bool mcu_flash_program(mcu_t *mcu, uint32_t offset, const void *data, size_t len);

const char *mcu_reset_cause_name(hal_reset_cause_t cause);

/* Binds the HAL implementation to an MCU instance (see hal_sim.c). */
void hal_sim_bind(mcu_t *mcu);
mcu_t *hal_sim_bound(void);

#ifdef __cplusplus
}
#endif

#endif /* SIM_MCU_H */
