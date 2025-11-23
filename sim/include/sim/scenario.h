/*
 * Scenario scripts: timed stimulus for the board.
 *
 * One event per line: "<time_ms> <command> [args...]". '#' starts a comment.
 *
 *   enable 0|1                 drive the enable pin
 *   estop 0|1                  assert / release the e-stop (active low pin)
 *   load <ohm>                 change load resistance
 *   vin <volt>                 change input voltage
 *   uart <text>                send text to the UART shell (\n, \r, \\ escapes;
 *                              a newline is appended if the text has none)
 *   can <id> [b0 b1 ...]       bench node transmits a raw frame (hex bytes)
 *   can_cmd vref <volt> | ilim <amp> | clear | reset | hang
 *                              convenience wrappers for the command protocol
 *   fault <name> [a] [b]       inject a fault (see faults.h)
 *   clear <name> [a] [b]       remove a fault
 *   mark <label>               start a named measurement window
 *   end                        stop the run at this time
 */
#ifndef SIM_SCENARIO_H
#define SIM_SCENARIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sim/board.h"
#include "sim/can.h"
#include "sim/clock.h"
#include "sim/faults.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SCENARIO_MAX_EVENTS 256
#define SCENARIO_TEXT_MAX 128

typedef enum {
    EV_ENABLE,
    EV_ESTOP,
    EV_LOAD,
    EV_VIN,
    EV_UART,
    EV_CAN,
    EV_FAULT,
    EV_CLEAR,
    EV_MARK,
    EV_END
} scenario_event_kind_t;

typedef struct {
    sim_time_us time_us;
    scenario_event_kind_t kind;
    double value;
    sim_fault_t fault;
    can_frame_t frame;
    char text[SCENARIO_TEXT_MAX];
    int line;
} scenario_event_t;

typedef struct {
    scenario_event_t events[SCENARIO_MAX_EVENTS];
    size_t count;
    size_t next;
    bool has_end;
    sim_time_us end_us;
} sim_scenario_t;

void scenario_init(sim_scenario_t *sc);
bool scenario_parse_string(sim_scenario_t *sc, const char *text, char *err, size_t err_len);
bool scenario_load_file(sim_scenario_t *sc, const char *path, char *err, size_t err_len);
bool scenario_parse_line(scenario_event_t *ev, const char *line, char *err, size_t err_len);

/* Applies every event due at or before "now" to the board. Calls on_event
 * (if non-NULL) for each applied event so the runner can log/mark it. */
typedef void (*scenario_event_cb)(void *ctx, const scenario_event_t *ev);
size_t scenario_apply_due(sim_scenario_t *sc, sim_board_t *b, scenario_event_cb cb, void *ctx);

const char *scenario_event_name(scenario_event_kind_t k);

#ifdef __cplusplus
}
#endif

#endif /* SIM_SCENARIO_H */
