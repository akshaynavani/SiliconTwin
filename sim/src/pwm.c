#include "sim/pwm.h"

#include <math.h>

void pwm_init(pwm_t *pwm, uint32_t frequency_hz, double duty_min, double duty_max)
{
    pwm->frequency_hz = frequency_hz;
    pwm->duty_min = duty_min;
    pwm->duty_max = duty_max;
    pwm->updates = 0;
    pwm->clamped_updates = 0;
    pwm_reset(pwm);
}

void pwm_reset(pwm_t *pwm)
{
    pwm->enabled = false;
    pwm->break_active = false;
    pwm->duty_cmd = 0.0;
}

void pwm_enable(pwm_t *pwm, bool enabled)
{
    pwm->enabled = enabled;
}

void pwm_set_duty(pwm_t *pwm, double duty)
{
    pwm->updates++;
    if (isnan(duty)) {
        duty = 0.0;
    }
    double clamped = duty;
    if (clamped > pwm->duty_max) {
        clamped = pwm->duty_max;
    }
    if (clamped < 0.0) {
        clamped = 0.0;
    }
    /* Pulses shorter than the minimum on-time are skipped entirely. */
    if (clamped > 0.0 && clamped < pwm->duty_min) {
        clamped = 0.0;
    }
    if (clamped != duty) {
        pwm->clamped_updates++;
    }
    pwm->duty_cmd = clamped;
}

void pwm_set_break(pwm_t *pwm, bool active)
{
    pwm->break_active = active;
}

double pwm_effective_duty(const pwm_t *pwm)
{
    if (!pwm->enabled || pwm->break_active) {
        return 0.0;
    }
    return pwm->duty_cmd;
}
