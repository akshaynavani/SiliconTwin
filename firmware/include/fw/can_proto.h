/*
 * CAN application protocol (11-bit identifiers, little-endian payloads).
 *
 *   0x080 + node  FAULT     event, sent on fault entry  [fault, state, vout_mV:u16, iout_mA:i16]
 *   0x100 + node  TELEM     periodic                    [vout_mV:u16, iout_mA:i16, vin_dV:u16, state, fault]
 *   0x180 + node  STATUS    every 100 ms                [temp_dC:i16, retries, reset_cause, wdt_resets, fault_hist, can_state, flags]
 *   0x200 + node  COMMAND   bench -> firmware           [cmd, arg_lo, arg_hi]
 *   0x280 + node  ACK       firmware -> bench           [cmd, status]
 *   0x700 + node  BOOT      sent once after every reset [reset_cause, boot_count:u16, params_ok]
 *
 * Lower identifiers win arbitration, so fault events pre-empt telemetry.
 */
#ifndef FW_CAN_PROTO_H
#define FW_CAN_PROTO_H

#include <stdbool.h>
#include <stdint.h>

#include "fw/hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FW_CAN_ID_FAULT   0x080u
#define FW_CAN_ID_TELEM   0x100u
#define FW_CAN_ID_STATUS  0x180u
#define FW_CAN_ID_COMMAND 0x200u
#define FW_CAN_ID_ACK     0x280u
#define FW_CAN_ID_BOOT    0x700u

typedef enum {
    FW_CMD_SET_VREF = 0x01,          /* arg: millivolts */
    FW_CMD_CLEAR_LATCH = 0x02,
    FW_CMD_SOFT_RESET = 0x03,
    FW_CMD_SET_CURRENT_LIMIT = 0x04, /* arg: milliamps */
    FW_CMD_HANG_TEST = 0x7E          /* deliberately stall the main loop (WDT test) */
} fw_can_cmd_t;

typedef enum {
    FW_ACK_OK = 0,
    FW_ACK_UNKNOWN = 1,
    FW_ACK_OUT_OF_RANGE = 2,
    FW_ACK_BAD_STATE = 3,
    FW_ACK_BAD_LENGTH = 4
} fw_can_ack_t;

typedef struct {
    uint16_t vout_mv;
    int16_t iout_ma;
    uint16_t vin_dv;
    uint8_t state;
    uint8_t fault;
} fw_telem_t;

void fw_put_u16(uint8_t *p, uint16_t v);
void fw_put_i16(uint8_t *p, int16_t v);
uint16_t fw_get_u16(const uint8_t *p);
int16_t fw_get_i16(const uint8_t *p);

/* Saturating float -> fixed-point helpers. */
uint16_t fw_to_u16(float value, float scale);
int16_t fw_to_i16(float value, float scale);

void fw_encode_telem(hal_can_frame_t *f, uint8_t node, const fw_telem_t *t);
bool fw_decode_telem(const hal_can_frame_t *f, fw_telem_t *t);

#ifdef __cplusplus
}
#endif

#endif /* FW_CAN_PROTO_H */
