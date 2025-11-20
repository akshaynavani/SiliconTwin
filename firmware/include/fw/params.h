/*
 * Firmware calibration / configuration parameters.
 *
 * Stored in the parameter flash page with a magic number, layout version and
 * CRC-32. If the page is blank or corrupt the firmware falls back to the
 * compiled-in defaults and reports it, so a bad calibration image can never
 * brick the controller.
 */
#ifndef FW_PARAMS_H
#define FW_PARAMS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_PARAMS_MAGIC 0x50325453u /* "ST2P" little-endian */
#define FW_PARAMS_VERSION 1u
#define FW_PARAMS_FLASH_OFFSET 0u

typedef struct {
    uint32_t magic;
    uint16_t version;
    uint16_t size;

    /* Regulation */
    float vref;            /* output voltage setpoint, V */
    float kp_v;            /* voltage loop proportional gain, A/V */
    float ki_v;            /* voltage loop integral gain, A/(V*s) */
    float kp_i;            /* current loop gain, V/A */
    float current_limit;   /* inductor current reference clamp, A */
    uint32_t soft_start_ms;

    /* Protection */
    float ocp_trip;        /* A, instantaneous over-current */
    float ovp_trip;        /* V, output over-voltage */
    float uvp_fraction;    /* under-voltage trip as a fraction of vref */
    float vin_min;         /* V */
    float vin_max;         /* V */
    float otp_trip;        /* deg C */
    float otp_clear;       /* deg C, hysteresis for restart */
    float sensor_tolerance;/* V, plausibility window for the vout sensor */
    uint32_t retry_delay_ms;
    uint8_t max_retries;

    /* Communication / supervision */
    uint8_t can_node_id;
    uint16_t telemetry_period_ms;
    uint32_t wdt_timeout_ms;

    uint32_t crc;          /* CRC-32 over all preceding bytes */
} fw_params_t;

void fw_params_defaults(fw_params_t *p);
/* Fills magic/version/size and computes the CRC. */
void fw_params_seal(fw_params_t *p);
bool fw_params_valid(const fw_params_t *p);
/* Range-checks values; invalid values make the whole block fall back. */
bool fw_params_sane(const fw_params_t *p);

uint32_t fw_crc32(const void *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FW_PARAMS_H */
