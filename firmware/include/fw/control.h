/*
 * Cascaded buck converter control.
 *
 *   outer loop: PI on output voltage error  -> inductor current reference
 *   inner loop: P on current error with Vout feed-forward -> duty cycle
 *
 * The current reference is clamped to the configured current limit, which
 * gives inherent cycle-by-cycle style current limiting: on overload or a
 * short the converter becomes a current source instead of tripping OCP.
 * The integrator uses conditional integration (clamping) for anti-windup.
 */
#ifndef FW_CONTROL_H
#define FW_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float kp;
    float ki;
    float ts;
    float integ;
    float out_min;
    float out_max;
    bool saturated;
} fw_pi_t;

void fw_pi_init(fw_pi_t *pi, float kp, float ki, float ts, float out_min, float out_max);
void fw_pi_reset(fw_pi_t *pi, float integ);
float fw_pi_step(fw_pi_t *pi, float error);

typedef struct {
    float vout;
    float iout;
    float vin;
    float temp;
} fw_meas_t;

typedef struct {
    fw_pi_t vloop;
    float kp_i;
    float r_series;   /* estimated series resistance for feed-forward */
    float duty_max;
    float i_ref;
    float duty;
} fw_ctrl_t;

void fw_ctrl_init(fw_ctrl_t *c, float kp_v, float ki_v, float kp_i, float current_limit,
                  float ts);
void fw_ctrl_set_current_limit(fw_ctrl_t *c, float current_limit);
void fw_ctrl_reset(fw_ctrl_t *c);
float fw_ctrl_step(fw_ctrl_t *c, float vref, const fw_meas_t *m);

/* Linear reference ramp used for soft start and setpoint changes. */
typedef struct {
    float value;
    float target;
    float step;       /* per control period */
} fw_ramp_t;

void fw_ramp_start(fw_ramp_t *r, float from, float to, uint32_t steps);
float fw_ramp_step(fw_ramp_t *r);
bool fw_ramp_done(const fw_ramp_t *r);

#ifdef __cplusplus
}
#endif

#endif /* FW_CONTROL_H */
