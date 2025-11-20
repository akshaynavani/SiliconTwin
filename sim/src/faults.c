#include "sim/faults.h"

#include <string.h>

static const char *const k_names[SIM_FAULT_COUNT] = {
    [SIM_FAULT_OUTPUT_SHORT] = "output_short",
    [SIM_FAULT_ADC_STUCK] = "adc_stuck",
    [SIM_FAULT_ADC_OFFSET] = "adc_offset",
    [SIM_FAULT_CAN_ERRORS] = "can_errors",
    [SIM_FAULT_CAN_BUS_OFF] = "can_bus_off",
    [SIM_FAULT_UART_NOISE] = "uart_noise",
    [SIM_FAULT_FW_HANG] = "fw_hang",
    [SIM_FAULT_GPIO_STUCK] = "gpio_stuck",
    [SIM_FAULT_BROWNOUT] = "brownout",
    [SIM_FAULT_OVERTEMP] = "overtemp",
};

const char *sim_fault_name(sim_fault_kind_t kind)
{
    if ((int)kind < 0 || kind >= SIM_FAULT_COUNT) {
        return "unknown";
    }
    return k_names[kind];
}

bool sim_fault_parse(const char *name, sim_fault_kind_t *out)
{
    for (int i = 0; i < SIM_FAULT_COUNT; i++) {
        if (strcmp(name, k_names[i]) == 0) {
            *out = (sim_fault_kind_t)i;
            return true;
        }
    }
    return false;
}
