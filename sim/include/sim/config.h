/*
 * Run configuration: board/bench settings, firmware parameter image and run
 * control, loaded from an INI file and overridable with "section.key=value"
 * strings (the Python harness uses these to sweep configuration matrices).
 *
 *   [sim]      name, duration_ms, seed, step_us, plant_substeps, trace_interval_us
 *   [plant]    vin, inductance_uh, capacitance_uf, r_inductor, r_load,
 *              t_ambient, r_thermal, tau_thermal
 *   [bench]    enable, uart_baud, can_bitrate, pwm_freq_hz, adc_noise_lsb
 *   [firmware] flash, corrupt_flash, vref, kp_v, ki_v, kp_i, current_limit,
 *              soft_start_ms, ocp_trip, ovp_trip, uvp_fraction, vin_min,
 *              vin_max, otp_trip, otp_clear, sensor_tolerance, retry_delay_ms,
 *              max_retries, can_node_id, telemetry_period_ms, wdt_timeout_ms
 */
#ifndef SIM_CONFIG_H
#define SIM_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fw/params.h"
#include "sim/board.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char name[64];
    board_config_t board;
    fw_params_t fw;
    bool flash_params;     /* program the parameter page (else firmware uses defaults) */
    bool corrupt_flash;    /* program it with a bad CRC to test the fallback */
    bool enable_at_start;  /* drive the enable pin high at t=0 */
    uint32_t duration_ms;
    uint32_t trace_interval_us;
} sim_config_t;

void sim_config_defaults(sim_config_t *cfg);

/* Sets one key. Returns false (and fills err) on unknown key or bad value. */
bool sim_config_set(sim_config_t *cfg, const char *section, const char *key, const char *value,
                    char *err, size_t err_len);
/* Parses "section.key=value". */
bool sim_config_apply_override(sim_config_t *cfg, const char *assignment, char *err,
                               size_t err_len);
bool sim_config_load_string(sim_config_t *cfg, const char *text, char *err, size_t err_len);
bool sim_config_load_file(sim_config_t *cfg, const char *path, char *err, size_t err_len);

/* Shared helper: reads a whole text file into a malloc'd, NUL-terminated buffer. */
char *sim_read_text_file(const char *path, size_t *len_out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_CONFIG_H */
