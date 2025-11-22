/*
 * Top level of the buck converter controller firmware.
 *
 * State machine:
 *
 *   STANDBY --enable--> SOFT_START --ramp done--> REGULATING
 *      ^                    |  \                      |
 *      |                fault   +------- fault -------+
 *      |                    v
 *      +--enable low---- FAULT --retry delay, retries left--> SOFT_START
 *      |                    |
 *      |              retries exhausted / latching fault
 *      |                    v
 *      +--enable low---- LATCHED            (any) --e-stop--> ESTOP
 *
 * Entry points: fw_init() after every reset, fw_main_tick() at 1 kHz.
 * The control loop runs in a 10 kHz interrupt registered through the HAL.
 */
#ifndef FW_FIRMWARE_H
#define FW_FIRMWARE_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/control.h"
#include "fw/hal.h"
#include "fw/params.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FW_CONTROL_RATE_HZ 10000u
#define FW_VERSION_STRING "silicontwin-buck 1.0.0"

typedef enum {
    FW_STATE_STANDBY = 0,
    FW_STATE_SOFT_START = 1,
    FW_STATE_REGULATING = 2,
    FW_STATE_FAULT = 3,
    FW_STATE_LATCHED = 4,
    FW_STATE_ESTOP = 5
} fw_state_t;

typedef struct {
    fw_state_t state;
    uint8_t active_fault;     /* fault that caused the current FAULT/LATCHED */
    uint8_t fault_history;    /* OR of every fault since power-on */
    uint8_t retries;
    uint32_t fault_count;
    float vref_target;
    float vref_now;
    float current_limit;
    float duty;
    float i_ref;
    fw_meas_t meas;
    bool pgood;
    bool params_from_flash;
    hal_reset_cause_t reset_cause;
    uint32_t boot_count;
    uint32_t wdt_resets;
    uint32_t can_commands;
    uint32_t can_bus_off_recoveries;
    uint32_t telemetry_frames;
    uint32_t state_entered_ms;
} fw_status_t;

void fw_init(void);
void fw_main_tick(void);

const fw_status_t *fw_status(void);
const fw_params_t *fw_active_params(void);
const char *fw_state_name(fw_state_t s);

/* Commands shared by the UART shell and the CAN interface. Return an
 * fw_can_ack_t status code. */
int fw_cmd_set_vref(float volts);
int fw_cmd_set_current_limit(float amps);
int fw_cmd_clear_latch(void);
void fw_cmd_soft_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* FW_FIRMWARE_H */
