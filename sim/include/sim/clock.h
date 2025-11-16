/*
 * Virtual simulation clock.
 *
 * All simulated time is expressed in integer microseconds so runs are exactly
 * reproducible regardless of host speed or floating-point accumulation.
 */
#ifndef SIM_CLOCK_H
#define SIM_CLOCK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint64_t sim_time_us;

#define SIM_US_PER_MS 1000ULL
#define SIM_MS(ms) ((sim_time_us)(ms) * SIM_US_PER_MS)

typedef struct {
    sim_time_us now;
} sim_clock_t;

/* A periodic deadline, e.g. a timer interrupt or a scheduled task. */
typedef struct {
    sim_time_us period;
    sim_time_us next;
    bool enabled;
} sim_periodic_t;

void sim_clock_init(sim_clock_t *clk);
void sim_clock_advance(sim_clock_t *clk, sim_time_us delta);
double sim_clock_seconds(const sim_clock_t *clk);

void sim_periodic_start(sim_periodic_t *p, sim_time_us now, sim_time_us period);
void sim_periodic_stop(sim_periodic_t *p);
/* Returns true (and re-arms) when the deadline has been reached. */
bool sim_periodic_due(sim_periodic_t *p, sim_time_us now);

#ifdef __cplusplus
}
#endif

#endif /* SIM_CLOCK_H */
