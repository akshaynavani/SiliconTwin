/*
 * Protection logic.
 *
 * Fast checks run in the 10 kHz control interrupt (over-current and
 * over-voltage must act within one switching-control period). Slow checks run
 * in the 1 kHz main loop with debounce timers (input voltage window, output
 * under-voltage, over-temperature, and a model-based plausibility check of the
 * output voltage sensor).
 */
#ifndef FW_PROTECTION_H
#define FW_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/control.h"
#include "fw/params.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Fault bits (also used on CAN and in retained RAM). */
#define FW_FAULT_NONE     0x00u
#define FW_FAULT_OCP      0x01u
#define FW_FAULT_OVP      0x02u
#define FW_FAULT_UVP      0x04u
#define FW_FAULT_VIN_UV   0x08u
#define FW_FAULT_VIN_OV   0x10u
#define FW_FAULT_OTP      0x20u
#define FW_FAULT_SENSOR   0x40u
#define FW_FAULT_WATCHDOG 0x80u

/* Debounce times (main-loop ticks = ms). */
#define FW_PROT_UVP_MS    20u
#define FW_PROT_VIN_MS    5u
#define FW_PROT_OTP_MS    10u
#define FW_PROT_SENSOR_MS 10u
#define FW_PROT_VIN_HYST  1.0f

typedef enum {
    FW_RECOVERY_RETRY,       /* auto-restart, consumes a retry */
    FW_RECOVERY_WAIT_CLEAR,  /* auto-restart once the condition clears, free */
    FW_RECOVERY_LATCH        /* never auto-restart */
} fw_recovery_t;

typedef struct {
    const fw_params_t *params;
    uint16_t uvp_count;
    uint16_t vin_uv_count;
    uint16_t vin_ov_count;
    uint16_t otp_count;
    uint16_t sensor_count;
    float vout_filt;
    float vest_filt;
} fw_prot_t;

void fw_prot_init(fw_prot_t *p, const fw_params_t *params);
void fw_prot_reset_counters(fw_prot_t *p);

/* Runs in the control ISR while switching. Returns fault bits (0 if none). */
uint8_t fw_prot_fast(fw_prot_t *p, const fw_meas_t *m);

/* Runs every 1 ms while switching. regulating: soft start has completed. */
uint8_t fw_prot_slow(fw_prot_t *p, const fw_meas_t *m, float duty, float vref, bool regulating);

/* Pre-start check of conditions that block switching (vin window, temp). */
uint8_t fw_prot_start_inhibit(const fw_prot_t *p, const fw_meas_t *m);

fw_recovery_t fw_prot_recovery(uint8_t fault);
/* For WAIT_CLEAR faults: has the triggering condition gone away? */
bool fw_prot_condition_cleared(const fw_prot_t *p, uint8_t fault, const fw_meas_t *m);

const char *fw_fault_name(uint8_t fault);

#ifdef __cplusplus
}
#endif

#endif /* FW_PROTECTION_H */
