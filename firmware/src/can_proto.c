#include "fw/can_proto.h"

#include <string.h>

void fw_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}

void fw_put_i16(uint8_t *p, int16_t v)
{
    fw_put_u16(p, (uint16_t)v);
}

uint16_t fw_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | (p[1] << 8));
}

int16_t fw_get_i16(const uint8_t *p)
{
    return (int16_t)fw_get_u16(p);
}

uint16_t fw_to_u16(float value, float scale)
{
    float x = value * scale + 0.5f;
    if (!(x > 0.0f)) {
        return 0;
    }
    if (x >= 65535.0f) {
        return 65535u;
    }
    return (uint16_t)x;
}

int16_t fw_to_i16(float value, float scale)
{
    float x = value * scale;
    x += (x >= 0.0f) ? 0.5f : -0.5f;
    if (x >= 32767.0f) {
        return 32767;
    }
    if (x <= -32768.0f) {
        return -32768;
    }
    if (x != x) {
        return 0;
    }
    return (int16_t)x;
}

void fw_encode_telem(hal_can_frame_t *f, uint8_t node, const fw_telem_t *t)
{
    memset(f, 0, sizeof(*f));
    f->id = (uint16_t)(FW_CAN_ID_TELEM + node);
    f->dlc = 8;
    fw_put_u16(&f->data[0], t->vout_mv);
    fw_put_i16(&f->data[2], t->iout_ma);
    fw_put_u16(&f->data[4], t->vin_dv);
    f->data[6] = t->state;
    f->data[7] = t->fault;
}

bool fw_decode_telem(const hal_can_frame_t *f, fw_telem_t *t)
{
    if ((f->id & 0x780u) != FW_CAN_ID_TELEM || f->dlc != 8) {
        return false;
    }
    t->vout_mv = fw_get_u16(&f->data[0]);
    t->iout_ma = fw_get_i16(&f->data[2]);
    t->vin_dv = fw_get_u16(&f->data[4]);
    t->state = f->data[6];
    t->fault = f->data[7];
    return true;
}
