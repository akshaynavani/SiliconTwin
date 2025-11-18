/*
 * Averaged model of a synchronous buck converter plus a first-order thermal
 * model of the switching stage.
 *
 *   L di/dt = d*Vin - v - i*R_L
 *   C dv/dt = i - v/R_load
 *   tau dT/dt = (T_amb + R_th*P_loss) - T
 *
 * Integrated with fixed-step forward Euler at a step small relative to the
 * LC resonance, which is plenty accurate for control-loop validation.
 */
#ifndef SIM_PLANT_H
#define SIM_PLANT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    double vin;          /* input voltage, V */
    double inductance;   /* H */
    double capacitance;  /* F */
    double r_inductor;   /* inductor DCR + switch resistance, ohm */
    double r_load;       /* nominal load, ohm */
    double t_ambient;    /* deg C */
    double r_thermal;    /* deg C per W */
    double tau_thermal;  /* s */
    double k_switching;  /* switching loss coefficient, W per (V*A) */
} plant_params_t;

typedef struct {
    plant_params_t p;
    double i_l;      /* inductor current, A */
    double v_out;    /* output voltage, V */
    double temp;     /* switch temperature, deg C */
    double r_load;   /* present load (may be changed by scenarios) */
    bool shorted;    /* fault: output short circuit */
    double r_short;
    double p_loss;
    double energy_out_j;
} plant_t;

void plant_default_params(plant_params_t *p);
void plant_init(plant_t *plant, const plant_params_t *params);
void plant_step(plant_t *plant, double duty, double dt);

void plant_set_load(plant_t *plant, double r_load);
void plant_set_vin(plant_t *plant, double vin);
void plant_set_ambient(plant_t *plant, double t_ambient);
void plant_set_short(plant_t *plant, bool shorted, double r_short);
double plant_effective_load(const plant_t *plant);

#ifdef __cplusplus
}
#endif

#endif /* SIM_PLANT_H */
