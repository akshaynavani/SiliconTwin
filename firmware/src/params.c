#include "fw/params.h"

#include <string.h>

uint32_t fw_crc32(const void *data, size_t len)
{
    /* Bitwise CRC-32 (IEEE 802.3, reflected). Small and table-free, which is
     * what a bootloader-adjacent routine on a small MCU would use. */
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1u));
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

void fw_params_defaults(fw_params_t *p)
{
    memset(p, 0, sizeof(*p));
    p->vref = 12.0f;
    p->kp_v = 3.0f;
    p->ki_v = 1500.0f;
    p->kp_i = 0.45f;
    p->current_limit = 8.0f;
    p->soft_start_ms = 20;

    p->ocp_trip = 12.0f;
    p->ovp_trip = 14.5f;
    p->uvp_fraction = 0.7f;
    p->vin_min = 20.0f;
    p->vin_max = 60.0f;
    p->otp_trip = 110.0f;
    p->otp_clear = 90.0f;
    p->sensor_tolerance = 3.0f;
    p->retry_delay_ms = 100;
    p->max_retries = 3;

    p->can_node_id = 1;
    p->telemetry_period_ms = 10;
    p->wdt_timeout_ms = 50;
    fw_params_seal(p);
}

void fw_params_seal(fw_params_t *p)
{
    p->magic = FW_PARAMS_MAGIC;
    p->version = FW_PARAMS_VERSION;
    p->size = (uint16_t)sizeof(*p);
    p->crc = fw_crc32(p, offsetof(fw_params_t, crc));
}

bool fw_params_valid(const fw_params_t *p)
{
    if (p->magic != FW_PARAMS_MAGIC || p->version != FW_PARAMS_VERSION ||
        p->size != sizeof(*p)) {
        return false;
    }
    return p->crc == fw_crc32(p, offsetof(fw_params_t, crc));
}

bool fw_params_sane(const fw_params_t *p)
{
    /* Written as "!(x within range)" so NaN fails every check. */
    if (!(p->vref > 0.5f && p->vref < 28.0f)) return false;
    if (!(p->kp_v > 0.0f && p->kp_v < 50.0f)) return false;
    if (!(p->ki_v >= 0.0f && p->ki_v < 1e5f)) return false;
    if (!(p->kp_i > 0.0f && p->kp_i < 5.0f)) return false;
    if (!(p->current_limit > 0.1f && p->current_limit < p->ocp_trip)) return false;
    if (!(p->ocp_trip > 0.0f && p->ocp_trip < 25.0f)) return false;
    if (!(p->ovp_trip > p->vref && p->ovp_trip < 30.0f)) return false;
    if (!(p->uvp_fraction > 0.0f && p->uvp_fraction < 1.0f)) return false;
    if (!(p->vin_min >= 0.0f && p->vin_max > p->vin_min && p->vin_max <= 100.0f)) return false;
    if (!(p->otp_clear < p->otp_trip && p->otp_trip < 200.0f)) return false;
    if (!(p->sensor_tolerance > 0.0f)) return false;
    if (p->soft_start_ms > 10000u || p->retry_delay_ms > 60000u) return false;
    if (p->can_node_id == 0 || p->can_node_id > 0x7Fu) return false;
    if (p->telemetry_period_ms == 0) return false;
    if (p->wdt_timeout_ms < 5u || p->wdt_timeout_ms > 10000u) return false;
    return true;
}
