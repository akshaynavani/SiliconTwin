/*
 * PWM timer model for the converter's switching stage.
 *
 * The plant uses an averaged model, so the PWM is represented by its duty
 * cycle. The hardware enforces a maximum duty (bootstrap refresh) and a
 * dead-time-equivalent minimum on-time; the outputs are forced off whenever
 * the timer is disabled or the hardware break input (e-stop) is asserted.
 */
#ifndef SIM_PWM_H
#define SIM_PWM_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frequency_hz;
    bool enabled;
    bool break_active; /* hardware break: forces outputs off */
    double duty_cmd;
    double duty_max;
    double duty_min;
    uint32_t updates;
    uint32_t clamped_updates;
} pwm_t;

void pwm_init(pwm_t *pwm, uint32_t frequency_hz, double duty_min, double duty_max);
void pwm_reset(pwm_t *pwm);
void pwm_enable(pwm_t *pwm, bool enabled);
void pwm_set_duty(pwm_t *pwm, double duty);
void pwm_set_break(pwm_t *pwm, bool active);
/* Duty actually applied to the power stage. */
double pwm_effective_duty(const pwm_t *pwm);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PWM_H */
