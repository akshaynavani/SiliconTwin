#include "sim/rng.h"

#include <math.h>

void sim_rng_seed(sim_rng_t *rng, uint64_t seed)
{
    /* xorshift must never hold an all-zero state. */
    rng->state = seed ? seed : 0x9E3779B97F4A7C15ULL;
    rng->has_spare = 0;
    rng->spare = 0.0;
}

uint64_t sim_rng_next(sim_rng_t *rng)
{
    uint64_t x = rng->state;
    x ^= x >> 12;
    x ^= x << 25;
    x ^= x >> 27;
    rng->state = x;
    return x * 0x2545F4914F6CDD1DULL;
}

double sim_rng_uniform(sim_rng_t *rng)
{
    /* Use the top 53 bits for a uniformly distributed double. */
    return (double)(sim_rng_next(rng) >> 11) * (1.0 / 9007199254740992.0);
}

double sim_rng_gaussian(sim_rng_t *rng)
{
    if (rng->has_spare) {
        rng->has_spare = 0;
        return rng->spare;
    }
    double u1;
    do {
        u1 = sim_rng_uniform(rng);
    } while (u1 <= 1e-300);
    double u2 = sim_rng_uniform(rng);
    double mag = sqrt(-2.0 * log(u1));
    const double two_pi = 6.283185307179586;
    rng->spare = mag * sin(two_pi * u2);
    rng->has_spare = 1;
    return mag * cos(two_pi * u2);
}

int sim_rng_chance(sim_rng_t *rng, double p)
{
    if (p <= 0.0) {
        return 0;
    }
    if (p >= 1.0) {
        return 1;
    }
    return sim_rng_uniform(rng) < p;
}
