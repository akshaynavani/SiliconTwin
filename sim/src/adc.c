#include "sim/adc.h"

#include <math.h>
#include <string.h>

static bool valid_ch(const adc_t *adc, uint8_t ch)
{
    return ch < adc->num_channels;
}

void adc_init(adc_t *adc, uint8_t num_channels, uint8_t bits, sim_rng_t *rng)
{
    memset(adc, 0, sizeof(*adc));
    adc->num_channels = num_channels > ADC_MAX_CHANNELS ? ADC_MAX_CHANNELS : num_channels;
    adc->bits = bits;
    adc->max_code = (uint16_t)((1u << bits) - 1u);
    adc->rng = rng;
    for (uint8_t i = 0; i < adc->num_channels; i++) {
        adc->ch[i].full_scale = 1.0;
    }
}

void adc_set_full_scale(adc_t *adc, uint8_t ch, double full_scale)
{
    if (valid_ch(adc, ch) && full_scale > 0.0) {
        adc->ch[ch].full_scale = full_scale;
    }
}

void adc_set_input(adc_t *adc, uint8_t ch, double value)
{
    if (valid_ch(adc, ch)) {
        adc->ch[ch].input = value;
    }
}

void adc_set_noise(adc_t *adc, double noise_lsb)
{
    adc->noise_lsb = noise_lsb < 0.0 ? 0.0 : noise_lsb;
}

uint16_t adc_convert(adc_t *adc, uint8_t ch)
{
    if (!valid_ch(adc, ch)) {
        return 0;
    }
    adc_channel_t *c = &adc->ch[ch];
    adc->conversions++;
    if (c->stuck) {
        c->last_code = c->stuck_code;
        return c->stuck_code;
    }
    double code = c->input / c->full_scale * (double)adc->max_code + c->offset_lsb;
    if (adc->rng && adc->noise_lsb > 0.0) {
        code += sim_rng_gaussian(adc->rng) * adc->noise_lsb;
    }
    code = floor(code + 0.5);
    if (code < 0.0) {
        code = 0.0;
    } else if (code > (double)adc->max_code) {
        code = (double)adc->max_code; /* input saturates the converter */
    }
    c->last_code = (uint16_t)code;
    return c->last_code;
}

void adc_inject_stuck(adc_t *adc, uint8_t ch, uint16_t code)
{
    if (valid_ch(adc, ch)) {
        adc->ch[ch].stuck = true;
        adc->ch[ch].stuck_code = code > adc->max_code ? adc->max_code : code;
    }
}

void adc_inject_offset(adc_t *adc, uint8_t ch, double offset_lsb)
{
    if (valid_ch(adc, ch)) {
        adc->ch[ch].offset_lsb = offset_lsb;
    }
}

void adc_clear_faults(adc_t *adc, uint8_t ch)
{
    if (valid_ch(adc, ch)) {
        adc->ch[ch].stuck = false;
        adc->ch[ch].offset_lsb = 0.0;
    }
}
