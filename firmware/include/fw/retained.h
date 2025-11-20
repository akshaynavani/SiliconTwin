/*
 * State kept in no-init RAM across warm resets. Lets the firmware count
 * watchdog resets, remember a latched fault (so a watchdog reset can never
 * be used to escape a latch-off) and detect reset loops.
 */
#ifndef FW_RETAINED_H
#define FW_RETAINED_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_RETAINED_MAGIC 0x52544E44u

typedef struct {
    uint32_t magic;
    uint32_t boot_count;
    uint32_t wdt_resets;
    uint32_t sw_resets;
    uint32_t brownouts;
    uint8_t consecutive_wdt;  /* watchdog resets without a healthy interval */
    uint8_t latched;          /* non-zero: latch-off must persist */
    uint8_t last_fault;
    uint8_t fault_history;
    uint32_t crc;
} fw_retained_t;

/* Returns a pointer into retained RAM; *valid tells whether it held data. */
fw_retained_t *fw_retained_attach(bool *valid);
void fw_retained_format(fw_retained_t *r);
void fw_retained_commit(fw_retained_t *r);

#ifdef __cplusplus
}
#endif

#endif /* FW_RETAINED_H */
