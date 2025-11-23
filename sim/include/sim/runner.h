/*
 * Runs one configuration + scenario through the board twin and collects the
 * metrics the regression harness asserts on.
 */
#ifndef SIM_RUNNER_H
#define SIM_RUNNER_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "fw/firmware.h"
#include "sim/board.h"
#include "sim/config.h"
#include "sim/scenario.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RUN_MAX_TRANSITIONS 128
#define RUN_MAX_FAULTS 64
#define RUN_MAX_WINDOWS 16
#define RUN_MAX_ACKS 32
#define RUN_MAX_RESETS 32
#define RUN_TAIL_SAMPLES 200 /* 20 ms at 100 us */

typedef struct {
    double t_ms;
    fw_state_t from;
    fw_state_t to;
} run_transition_t;

typedef struct {
    double t_ms;
    uint8_t fault;
} run_fault_t;

typedef struct {
    double t_ms;
    hal_reset_cause_t cause;
} run_reset_t;

typedef struct {
    double t_ms;
    uint8_t cmd;
    uint8_t status;
} run_ack_t;

typedef struct {
    char label[32];
    double start_ms;
    double end_ms;
    double vout_min;
    double vout_max;
    double iout_max;
    double settle_ms;   /* -1 when the output never settled in the window */
    sim_time_us last_out_of_band;
    bool in_band;
    bool open;
} run_window_t;

typedef struct {
    char name[64];
    double sim_time_ms;
    uint64_t steps;

    /* Regulation */
    double vout_final, iout_final, vin_final, vref_final;
    double vout_min, vout_max, iout_peak, temp_peak, energy_j;
    double startup_ms;      /* first enable -> first PGOOD, -1 if never */
    double overshoot_pct;   /* during start-up */
    double steady_state_error;
    double ripple_pp;
    bool pgood_final;
    fw_state_t final_state;
    uint8_t final_fault;
    uint8_t fault_history;
    uint8_t retries_final;
    bool params_from_flash;

    run_transition_t transitions[RUN_MAX_TRANSITIONS];
    uint32_t n_transitions;
    run_fault_t faults[RUN_MAX_FAULTS];
    uint32_t n_faults;
    run_window_t windows[RUN_MAX_WINDOWS];
    uint32_t n_windows;
    run_ack_t acks[RUN_MAX_ACKS];
    uint32_t n_acks;
    run_reset_t resets[RUN_MAX_RESETS]; /* warm resets only (power-on excluded) */
    uint32_t n_resets;

    /* Watchdog */
    uint32_t wdt_expirations;
    uint32_t wdt_kicks;
    double wdt_max_kick_gap_ms;

    /* CAN */
    uint32_t can_frames_ok;
    uint32_t can_error_frames;
    uint32_t can_bus_off_events;
    uint32_t can_max_tec;
    uint32_t rx_telemetry, rx_status, rx_fault, rx_boot, rx_ack;
    can_state_t can_final_state;

    /* UART */
    char *uart_output; /* malloc'd copy of everything the firmware printed */
    uart_stats_t uart_stats;

    uint32_t events_applied;
} sim_metrics_t;

/* Runs to completion. trace may be NULL. Returns false on setup errors. */
bool sim_run(const sim_config_t *cfg, sim_scenario_t *sc, sim_metrics_t *m, FILE *trace,
             char *err, size_t err_len);
void sim_metrics_free(sim_metrics_t *m);
void sim_metrics_write_json(const sim_metrics_t *m, FILE *out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_RUNNER_H */
