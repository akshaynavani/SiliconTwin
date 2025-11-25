#include "fw/protection.h"

#include "fw/hal.h"

static uint8_t debounce(uint16_t *counter, bool condition, uint16_t limit, uint8_t fault)
{
    if (!condition) {
        *counter = 0;
        return 0;
    }
    if (*counter < 0xFFFFu) {
        (*counter)++;
    }
    return *counter >= limit ? fault : 0;
}

void fw_prot_init(fw_prot_t *p, const fw_params_t *params)
{
    p->params = params;
    fw_prot_reset_counters(p);
}

void fw_prot_reset_counters(fw_prot_t *p)
{
    p->uvp_count = 0;
    p->vin_uv_count = 0;
    p->vin_ov_count = 0;
    p->otp_count = 0;
    p->sensor_count = 0;
    p->vout_filt = 0.0f;
    p->vest_filt = 0.0f;
}

uint8_t fw_prot_fast(fw_prot_t *p, const fw_meas_t *m)
{
    uint8_t faults = 0;
    /* Acts like a hardware comparator: a single sample over the trip point. */
    if (m->iout > p->params->ocp_trip) {
        faults |= FW_FAULT_OCP;
    }
    if (m->vout > p->params->ovp_trip) {
        faults |= FW_FAULT_OVP;
    }
    return faults;
}

uint8_t fw_prot_slow(fw_prot_t *p, const fw_meas_t *m, float duty, float vref, bool regulating)
{
    const fw_params_t *prm = p->params;
    uint8_t faults = 0;

    faults |= debounce(&p->vin_uv_count, m->vin < prm->vin_min, FW_PROT_VIN_MS, FW_FAULT_VIN_UV);
    faults |= debounce(&p->vin_ov_count, m->vin > prm->vin_max, FW_PROT_VIN_MS, FW_FAULT_VIN_OV);
    faults |= debounce(&p->otp_count, m->temp > prm->otp_trip, FW_PROT_OTP_MS, FW_FAULT_OTP);

    if (regulating) {
        faults |= debounce(&p->uvp_count, m->vout < prm->uvp_fraction * vref, FW_PROT_UVP_MS,
                           FW_FAULT_UVP);
    } else {
        p->uvp_count = 0;
    }

    /* Sensor plausibility: in the averaged steady state the switch-node
     * voltage d*Vin minus the resistive drop must match the measured output.
     * Both sides are low-pass filtered (tau ~ 1.5 ms) to ignore LC transients. */
    const float alpha = 0.5f;
    float v_est = duty * m->vin - m->iout * 0.05f;
    p->vout_filt += alpha * (m->vout - p->vout_filt);
    p->vest_filt += alpha * (v_est - p->vest_filt);
    float diff = p->vout_filt - p->vest_filt;
    if (diff < 0.0f) {
        diff = -diff;
    }
    /* A sense line pinned at either ADC rail while regulating is an open or
     * shorted divider: a real output never sits at exactly 0 V or full scale. */
    bool at_rail = m->vout <= 0.0f || m->vout >= HAL_ADC_VOUT_FULL_SCALE - 0.01f;
    bool implausible = diff > prm->sensor_tolerance || at_rail;
    faults |= debounce(&p->sensor_count, regulating && implausible,
                       FW_PROT_SENSOR_MS, FW_FAULT_SENSOR);
    return faults;
}

uint8_t fw_prot_start_inhibit(const fw_prot_t *p, const fw_meas_t *m)
{
    const fw_params_t *prm = p->params;
    uint8_t f = 0;
    if (m->vin < prm->vin_min) {
        f |= FW_FAULT_VIN_UV;
    }
    if (m->vin > prm->vin_max) {
        f |= FW_FAULT_VIN_OV;
    }
    if (m->temp > prm->otp_clear) {
        f |= FW_FAULT_OTP;
    }
    return f;
}

fw_recovery_t fw_prot_recovery(uint8_t fault)
{
    switch (fault) {
    case FW_FAULT_OCP:
    case FW_FAULT_OVP:
    case FW_FAULT_UVP:
        return FW_RECOVERY_RETRY;
    case FW_FAULT_VIN_UV:
    case FW_FAULT_VIN_OV:
    case FW_FAULT_OTP:
        return FW_RECOVERY_WAIT_CLEAR;
    default:
        return FW_RECOVERY_LATCH;
    }
}

bool fw_prot_condition_cleared(const fw_prot_t *p, uint8_t fault, const fw_meas_t *m)
{
    const fw_params_t *prm = p->params;
    switch (fault) {
    case FW_FAULT_VIN_UV:
        return m->vin > prm->vin_min + FW_PROT_VIN_HYST;
    case FW_FAULT_VIN_OV:
        return m->vin < prm->vin_max - FW_PROT_VIN_HYST;
    case FW_FAULT_OTP:
        return m->temp < prm->otp_clear;
    default:
        return true;
    }
}

const char *fw_fault_name(uint8_t fault)
{
    switch (fault) {
    case FW_FAULT_NONE: return "NONE";
    case FW_FAULT_OCP: return "OCP";
    case FW_FAULT_OVP: return "OVP";
    case FW_FAULT_UVP: return "UVP";
    case FW_FAULT_VIN_UV: return "VIN_UV";
    case FW_FAULT_VIN_OV: return "VIN_OV";
    case FW_FAULT_OTP: return "OTP";
    case FW_FAULT_SENSOR: return "SENSOR";
    case FW_FAULT_WATCHDOG: return "WATCHDOG";
    default: return "MULTI";
    }
}
