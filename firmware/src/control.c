#include "fw/control.h"

static float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

void fw_pi_init(fw_pi_t *pi, float kp, float ki, float ts, float out_min, float out_max)
{
    pi->kp = kp;
    pi->ki = ki;
    pi->ts = ts;
    pi->out_min = out_min;
    pi->out_max = out_max;
    fw_pi_reset(pi, 0.0f);
}

void fw_pi_reset(fw_pi_t *pi, float integ)
{
    pi->integ = clampf(integ, pi->out_min, pi->out_max);
    pi->saturated = false;
}

float fw_pi_step(fw_pi_t *pi, float error)
{
    float p = pi->kp * error;
    float candidate = pi->integ + pi->ki * pi->ts * error;
    float out = p + candidate;

    /* Conditional integration: only accept the new integrator value if it
     * does not push further into saturation. */
    bool high = out > pi->out_max;
    bool low = out < pi->out_min;
    pi->saturated = high || low;
    if (!((high && error > 0.0f) || (low && error < 0.0f))) {
        pi->integ = clampf(candidate, pi->out_min, pi->out_max);
    }
    return clampf(p + pi->integ, pi->out_min, pi->out_max);
}

void fw_ctrl_init(fw_ctrl_t *c, float kp_v, float ki_v, float kp_i, float current_limit,
                  float ts)
{
    /* Allow a small negative current so the synchronous stage can pull the
     * output down after a setpoint decrease. */
    fw_pi_init(&c->vloop, kp_v, ki_v, ts, -1.0f, current_limit);
    c->kp_i = kp_i;
    c->r_series = 0.05f;
    c->duty_max = 0.95f;
    c->i_ref = 0.0f;
    c->duty = 0.0f;
}

void fw_ctrl_set_current_limit(fw_ctrl_t *c, float current_limit)
{
    c->vloop.out_max = current_limit;
    if (c->vloop.integ > current_limit) {
        c->vloop.integ = current_limit;
    }
}

void fw_ctrl_reset(fw_ctrl_t *c)
{
    fw_pi_reset(&c->vloop, 0.0f);
    c->i_ref = 0.0f;
    c->duty = 0.0f;
}

float fw_ctrl_step(fw_ctrl_t *c, float vref, const fw_meas_t *m)
{
    c->i_ref = fw_pi_step(&c->vloop, vref - m->vout);

    /* Inner loop: required inductor voltage plus feed-forward of the output
     * voltage and resistive drop, normalised by the input voltage. */
    float v_l = c->kp_i * (c->i_ref - m->iout);
    float v_sw = m->vout + m->iout * c->r_series + v_l;
    float duty = m->vin > 1.0f ? v_sw / m->vin : 0.0f;
    c->duty = clampf(duty, 0.0f, c->duty_max);
    return c->duty;
}

void fw_ramp_start(fw_ramp_t *r, float from, float to, uint32_t steps)
{
    r->value = from;
    r->target = to;
    if (steps == 0) {
        r->value = to;
        r->step = 0.0f;
        return;
    }
    r->step = (to - from) / (float)steps;
}

float fw_ramp_step(fw_ramp_t *r)
{
    if (fw_ramp_done(r)) {
        r->value = r->target;
        return r->value;
    }
    r->value += r->step;
    if ((r->step > 0.0f && r->value > r->target) || (r->step < 0.0f && r->value < r->target)) {
        r->value = r->target;
    }
    return r->value;
}

bool fw_ramp_done(const fw_ramp_t *r)
{
    return r->step == 0.0f || r->value == r->target;
}
