#include "sim/watchdog.h"

#include <string.h>

void wdt_init(watchdog_t *w)
{
    memset(w, 0, sizeof(*w));
}

void wdt_reset(watchdog_t *w)
{
    w->running = false;
}

void wdt_start(watchdog_t *w, sim_time_us now, sim_time_us timeout_us)
{
    if (w->running || timeout_us == 0) {
        return; /* the timeout is write-once until the next reset */
    }
    w->running = true;
    w->timeout_us = timeout_us;
    w->deadline = now + timeout_us;
    w->last_kick = now;
}

void wdt_kick(watchdog_t *w, sim_time_us now)
{
    if (!w->running) {
        return;
    }
    sim_time_us gap = now - w->last_kick;
    if (gap > w->max_kick_interval) {
        w->max_kick_interval = gap;
    }
    w->last_kick = now;
    w->deadline = now + w->timeout_us;
    w->kicks++;
}

bool wdt_step(watchdog_t *w, sim_time_us now)
{
    if (!w->running || now < w->deadline) {
        return false;
    }
    w->running = false;
    w->expirations++;
    return true;
}
