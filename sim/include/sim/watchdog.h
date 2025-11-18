/*
 * Independent watchdog timer.
 *
 * Once started it cannot be stopped by software; if it is not kicked within
 * the timeout it expires and the MCU model performs a watchdog reset.
 */
#ifndef SIM_WATCHDOG_H
#define SIM_WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

#include "sim/clock.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool running;
    sim_time_us timeout_us;
    sim_time_us deadline;
    sim_time_us last_kick;
    sim_time_us max_kick_interval; /* worst observed gap between kicks */
    uint32_t kicks;
    uint32_t expirations;
} watchdog_t;

void wdt_init(watchdog_t *w);
/* Stops the timer as part of an MCU reset (statistics are preserved). */
void wdt_reset(watchdog_t *w);
void wdt_start(watchdog_t *w, sim_time_us now, sim_time_us timeout_us);
void wdt_kick(watchdog_t *w, sim_time_us now);
/* Returns true exactly once when the watchdog expires. */
bool wdt_step(watchdog_t *w, sim_time_us now);

#ifdef __cplusplus
}
#endif

#endif /* SIM_WATCHDOG_H */
