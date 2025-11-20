/*
 * The complete "digital twin" of the target board: one MCU running firmware,
 * the converter power stage, and a CAN bus with a test-bench node attached.
 *
 * board_step() advances the world by one simulation step:
 *   1. integrate the power stage using the PWM duty currently applied,
 *   2. advance the clock and sample the analog world into the ADC inputs,
 *   3. clock the communication peripherals (UART, CAN),
 *   4. let the MCU run its due interrupts and super-loop tick.
 */
#ifndef SIM_BOARD_H
#define SIM_BOARD_H

#include <stdbool.h>
#include <stdint.h>

#include "sim/can.h"
#include "sim/clock.h"
#include "sim/faults.h"
#include "sim/mcu.h"
#include "sim/plant.h"
#include "sim/rng.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    plant_params_t plant;
    uint64_t seed;
    uint32_t step_us;          /* simulation step, default 10 us */
    uint32_t plant_substeps;   /* Euler sub-steps per sim step, default 10 */
    uint32_t uart_baud;
    uint32_t can_bitrate;
    uint32_t pwm_freq_hz;
    double adc_noise_lsb;
} board_config_t;

typedef struct {
    board_config_t cfg;
    sim_clock_t clock;
    sim_rng_t rng;
    plant_t plant;
    can_bus_t can;
    mcu_t mcu;
    uint8_t mcu_node;
    uint8_t bench_node;
    bool active_faults[SIM_FAULT_COUNT];
    uint32_t fault_injections;
} sim_board_t;

void board_default_config(board_config_t *cfg);
void board_init(sim_board_t *b, const board_config_t *cfg, const mcu_firmware_t *fw);
void board_power_on(sim_board_t *b);
void board_step(sim_board_t *b);
void board_run_for(sim_board_t *b, sim_time_us duration);
sim_time_us board_now(const sim_board_t *b);

/* Test-bench conveniences. */
void board_set_enable(sim_board_t *b, bool on);
void board_set_estop(sim_board_t *b, bool asserted);
bool board_bench_can_send(sim_board_t *b, uint16_t id, const uint8_t *data, uint8_t dlc);
bool board_bench_can_receive(sim_board_t *b, can_frame_t *frame);
size_t board_uart_send(sim_board_t *b, const char *text);

void board_inject_fault(sim_board_t *b, const sim_fault_t *fault);
void board_clear_fault(sim_board_t *b, const sim_fault_t *fault);

#ifdef __cplusplus
}
#endif

#endif /* SIM_BOARD_H */
