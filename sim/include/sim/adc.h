/*
 * Multi-channel successive-approximation ADC model.
 *
 * The board model writes the analog value present at each input; firmware
 * reads quantised codes. Gaussian noise (in LSB), offset errors and stuck
 * channels can be injected to exercise the firmware's plausibility checks.
 */
#ifndef SIM_ADC_H
#define SIM_ADC_H

#include <stdbool.h>
#include <stdint.h>

#include "sim/rng.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ADC_MAX_CHANNELS 8

typedef struct {
    double input;       /* analog value in engineering units */
    double full_scale;  /* value that maps to max code */
    double offset_lsb;  /* injected offset error */
    bool stuck;
    uint16_t stuck_code;
    uint16_t last_code;
} adc_channel_t;

typedef struct {
    adc_channel_t ch[ADC_MAX_CHANNELS];
    uint8_t num_channels;
    uint8_t bits;
    uint16_t max_code;
    double noise_lsb;   /* stddev of conversion noise */
    sim_rng_t *rng;
    uint32_t conversions;
} adc_t;

void adc_init(adc_t *adc, uint8_t num_channels, uint8_t bits, sim_rng_t *rng);
void adc_set_full_scale(adc_t *adc, uint8_t ch, double full_scale);
void adc_set_input(adc_t *adc, uint8_t ch, double value);
void adc_set_noise(adc_t *adc, double noise_lsb);
uint16_t adc_convert(adc_t *adc, uint8_t ch);

/* Fault injection. */
void adc_inject_stuck(adc_t *adc, uint8_t ch, uint16_t code);
void adc_inject_offset(adc_t *adc, uint8_t ch, double offset_lsb);
void adc_clear_faults(adc_t *adc, uint8_t ch);

#ifdef __cplusplus
}
#endif

#endif /* SIM_ADC_H */
