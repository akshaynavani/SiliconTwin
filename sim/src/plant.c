#include "sim/plant.h"

#include <string.h>

void plant_default_params(plant_params_t *p)
{
    p->vin = 48.0;
    p->inductance = 47e-6;
    p->capacitance = 470e-6;
    p->r_inductor = 0.05;
    p->r_load = 2.4;   /* 12 V at 5 A */
    p->t_ambient = 25.0;
    p->r_thermal = 4.0;
    p->tau_thermal = 2.0;
    p->k_switching = 0.004;
}

void plant_init(plant_t *plant, const plant_params_t *params)
{
    memset(plant, 0, sizeof(*plant));
    plant->p = *params;
    plant->r_load = params->r_load;
    plant->temp = params->t_ambient;
    plant->r_short = 0.02;
}

double plant_effective_load(const plant_t *plant)
{
    if (!plant->shorted) {
        return plant->r_load;
    }
    /* Short in parallel with the normal load. */
    return (plant->r_load * plant->r_short) / (plant->r_load + plant->r_short);
}

void plant_step(plant_t *plant, double duty, bool hiz, double dt)
{
    const plant_params_t *p = &plant->p;
    double r_load = plant_effective_load(plant);
    const double v_diode = 0.7;

    double v_sw;
    if (!hiz) {
        v_sw = duty * p->vin;
    } else if (plant->i_l > 0.0) {
        v_sw = -v_diode; /* low-side body diode conducts */
    } else if (plant->i_l < 0.0) {
        v_sw = p->vin + v_diode; /* high-side body diode returns energy to the input */
    } else {
        v_sw = plant->v_out; /* no current path: inductor stays at zero */
    }

    double di = (v_sw - plant->v_out - plant->i_l * p->r_inductor) / p->inductance;
    double i_load = r_load > 0.0 ? plant->v_out / r_load : 0.0;
    double dv = (plant->i_l - i_load) / p->capacitance;

    double i_prev = plant->i_l;
    plant->i_l += di * dt;
    if (hiz && ((i_prev > 0.0 && plant->i_l < 0.0) || (i_prev < 0.0 && plant->i_l > 0.0))) {
        plant->i_l = 0.0; /* diodes block reverse current */
    }
    plant->v_out += dv * dt;
    if (plant->v_out < 0.0) {
        /* Body diode of the low-side switch clamps the output near ground. */
        plant->v_out = 0.0;
    }

    double i_abs = plant->i_l < 0.0 ? -plant->i_l : plant->i_l;
    double conduction = plant->i_l * plant->i_l * p->r_inductor;
    double switching = (!hiz && duty > 0.0) ? p->k_switching * p->vin * i_abs : 0.0;
    plant->p_loss = conduction + switching;
    double t_target = p->t_ambient + p->r_thermal * plant->p_loss;
    plant->temp += (t_target - plant->temp) * dt / p->tau_thermal;

    plant->energy_out_j += plant->v_out * i_load * dt;
}

void plant_set_load(plant_t *plant, double r_load)
{
    if (r_load > 0.0) {
        plant->r_load = r_load;
    }
}

void plant_set_vin(plant_t *plant, double vin)
{
    plant->p.vin = vin < 0.0 ? 0.0 : vin;
}

void plant_set_ambient(plant_t *plant, double t_ambient)
{
    plant->p.t_ambient = t_ambient;
}

void plant_set_short(plant_t *plant, bool shorted, double r_short)
{
    plant->shorted = shorted;
    if (r_short > 0.0) {
        plant->r_short = r_short;
    }
}
