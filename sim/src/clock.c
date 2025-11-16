#include "sim/clock.h"

void sim_clock_init(sim_clock_t *clk)
{
    clk->now = 0;
}

void sim_clock_advance(sim_clock_t *clk, sim_time_us delta)
{
    clk->now += delta;
}

double sim_clock_seconds(const sim_clock_t *clk)
{
    return (double)clk->now / 1e6;
}

void sim_periodic_start(sim_periodic_t *p, sim_time_us now, sim_time_us period)
{
    p->period = period;
    p->next = now + period;
    p->enabled = period > 0;
}

void sim_periodic_stop(sim_periodic_t *p)
{
    p->enabled = false;
}

bool sim_periodic_due(sim_periodic_t *p, sim_time_us now)
{
    if (!p->enabled || now < p->next) {
        return false;
    }
    /* Re-arm relative to the schedule, not to "now", so the period does not drift. */
    p->next += p->period;
    if (p->next <= now) {
        /* We fell behind by more than one period; skip the missed ones. */
        p->next = now + p->period;
    }
    return true;
}
