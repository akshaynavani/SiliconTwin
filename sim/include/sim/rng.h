/*
 * Small deterministic PRNG (xorshift64*) used for sensor noise and fault
 * injection so that a given seed always reproduces the same run.
 */
#ifndef SIM_RNG_H
#define SIM_RNG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t state;
    int has_spare;
    double spare;
} sim_rng_t;

void sim_rng_seed(sim_rng_t *rng, uint64_t seed);
uint64_t sim_rng_next(sim_rng_t *rng);
/* Uniform in [0, 1). */
double sim_rng_uniform(sim_rng_t *rng);
/* Standard normal (mean 0, stddev 1) via Box-Muller. */
double sim_rng_gaussian(sim_rng_t *rng);
/* Returns 1 with probability p. */
int sim_rng_chance(sim_rng_t *rng, double p);

#ifdef __cplusplus
}
#endif

#endif /* SIM_RNG_H */
