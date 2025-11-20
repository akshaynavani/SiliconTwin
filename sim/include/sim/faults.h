/*
 * Fault injection catalogue.
 *
 * Each fault has up to two numeric arguments whose meaning depends on the
 * kind (see the table in faults.c). Faults are injected and cleared on a
 * sim_board_t, usually from a scenario script.
 */
#ifndef SIM_FAULTS_H
#define SIM_FAULTS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SIM_FAULT_OUTPUT_SHORT = 0, /* a: short resistance in ohm (default 0.02) */
    SIM_FAULT_ADC_STUCK,        /* a: channel, b: stuck code */
    SIM_FAULT_ADC_OFFSET,       /* a: channel, b: offset in LSB */
    SIM_FAULT_CAN_ERRORS,       /* a: per-frame error probability */
    SIM_FAULT_CAN_BUS_OFF,      /* force the MCU's CAN node into bus-off */
    SIM_FAULT_UART_NOISE,       /* a: per-byte corruption probability */
    SIM_FAULT_FW_HANG,          /* a: duration in ms, 0 = until reset */
    SIM_FAULT_GPIO_STUCK,       /* a: pin, b: level */
    SIM_FAULT_BROWNOUT,         /* instantaneous brown-out reset */
    SIM_FAULT_OVERTEMP,         /* a: ambient temperature in deg C */
    SIM_FAULT_COUNT
} sim_fault_kind_t;

typedef struct {
    sim_fault_kind_t kind;
    double a;
    double b;
} sim_fault_t;

const char *sim_fault_name(sim_fault_kind_t kind);
/* Parses a fault name (e.g. "output_short"). Returns false if unknown. */
bool sim_fault_parse(const char *name, sim_fault_kind_t *out);

#ifdef __cplusplus
}
#endif

#endif /* SIM_FAULTS_H */
