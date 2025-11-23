#include "sim/scenario.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fw/can_proto.h"
#include "sim/config.h"

void scenario_init(sim_scenario_t *sc)
{
    memset(sc, 0, sizeof(*sc));
}

static void fail(char *err, size_t len, const char *msg, const char *detail)
{
    if (err && len) {
        snprintf(err, len, "%s%s%s", msg, detail ? ": " : "", detail ? detail : "");
    }
}

static const char *skip_ws(const char *s)
{
    while (*s && isspace((unsigned char)*s)) {
        s++;
    }
    return s;
}

/* Copies the next whitespace-delimited token into out. Returns pointer past it. */
static const char *next_token(const char *s, char *out, size_t out_len)
{
    s = skip_ws(s);
    size_t n = 0;
    while (*s && !isspace((unsigned char)*s)) {
        if (n + 1 < out_len) {
            out[n++] = *s;
        }
        s++;
    }
    out[n] = '\0';
    return s;
}

static bool parse_num(const char *tok, double *out)
{
    if (!*tok) {
        return false;
    }
    char *end = NULL;
    double v = strtod(tok, &end);
    if (*end != '\0') {
        return false;
    }
    *out = v;
    return true;
}

static void unescape(const char *in, char *out, size_t out_len)
{
    size_t n = 0;
    /* Strip surrounding quotes if present. */
    size_t len = strlen(in);
    if (len >= 2 && in[0] == '"' && in[len - 1] == '"') {
        in++;
        len -= 2;
    }
    for (size_t i = 0; i < len && n + 1 < out_len; i++) {
        char c = in[i];
        if (c == '\\' && i + 1 < len) {
            char e = in[++i];
            c = e == 'n' ? '\n' : e == 'r' ? '\r' : e == 't' ? '\t' : e;
        }
        out[n++] = c;
    }
    out[n] = '\0';
    if (n == 0 || (out[n - 1] != '\n' && out[n - 1] != '\r')) {
        if (n + 1 < out_len) {
            out[n++] = '\n';
            out[n] = '\0';
        }
    }
}

static bool build_can_cmd(scenario_event_t *ev, const char *rest, char *err, size_t err_len)
{
    char sub[32], arg[32];
    rest = next_token(rest, sub, sizeof(sub));
    next_token(rest, arg, sizeof(arg));
    /* Bench addresses the default node; raw "can" frames can target others. */
    can_frame_t *f = &ev->frame;
    memset(f, 0, sizeof(*f));
    f->id = (uint16_t)(FW_CAN_ID_COMMAND + 1u);
    double v = 0.0;
    if (!strcmp(sub, "vref") || !strcmp(sub, "ilim")) {
        if (!parse_num(arg, &v) || v < 0.0 || v > 65.0) {
            fail(err, err_len, "can_cmd needs a numeric argument", sub);
            return false;
        }
        f->data[0] = !strcmp(sub, "vref") ? FW_CMD_SET_VREF : FW_CMD_SET_CURRENT_LIMIT;
        uint16_t milli = (uint16_t)(v * 1000.0 + 0.5);
        f->data[1] = (uint8_t)(milli & 0xFFu);
        f->data[2] = (uint8_t)(milli >> 8);
        f->dlc = 3;
    } else if (!strcmp(sub, "clear")) {
        f->data[0] = FW_CMD_CLEAR_LATCH;
        f->dlc = 1;
    } else if (!strcmp(sub, "reset")) {
        f->data[0] = FW_CMD_SOFT_RESET;
        f->dlc = 1;
    } else if (!strcmp(sub, "hang")) {
        f->data[0] = FW_CMD_HANG_TEST;
        f->dlc = 1;
    } else {
        fail(err, err_len, "unknown can_cmd", sub);
        return false;
    }
    ev->kind = EV_CAN;
    snprintf(ev->text, sizeof(ev->text), "can_cmd %s %s", sub, arg);
    return true;
}

bool scenario_parse_line(scenario_event_t *ev, const char *line, char *err, size_t err_len)
{
    memset(ev, 0, sizeof(*ev));
    char tok[64], cmd[32];
    const char *p = next_token(line, tok, sizeof(tok));
    double t;
    if (!parse_num(tok, &t) || t < 0.0) {
        fail(err, err_len, "expected a time in ms", tok);
        return false;
    }
    ev->time_us = (sim_time_us)(t * 1000.0 + 0.5);
    p = next_token(p, cmd, sizeof(cmd));

    char a[64] = "", b[64] = "", c[64] = "";
    if (!strcmp(cmd, "enable") || !strcmp(cmd, "estop") || !strcmp(cmd, "load") ||
        !strcmp(cmd, "vin")) {
        next_token(p, a, sizeof(a));
        if (!parse_num(a, &ev->value)) {
            fail(err, err_len, "expected a number after", cmd);
            return false;
        }
        ev->kind = !strcmp(cmd, "enable") ? EV_ENABLE
                 : !strcmp(cmd, "estop")  ? EV_ESTOP
                 : !strcmp(cmd, "load")   ? EV_LOAD
                                          : EV_VIN;
        if ((ev->kind == EV_LOAD && ev->value <= 0.0) || (ev->kind == EV_VIN && ev->value < 0.0)) {
            fail(err, err_len, "value out of range for", cmd);
            return false;
        }
        snprintf(ev->text, sizeof(ev->text), "%s %s", cmd, a);
        return true;
    }
    if (!strcmp(cmd, "uart")) {
        ev->kind = EV_UART;
        unescape(skip_ws(p), ev->text, sizeof(ev->text));
        return true;
    }
    if (!strcmp(cmd, "can")) {
        p = next_token(p, a, sizeof(a));
        char *end = NULL;
        unsigned long id = strtoul(a, &end, 0);
        if (!*a || *end != '\0' || id > CAN_ID_MASK) {
            fail(err, err_len, "bad CAN identifier", a);
            return false;
        }
        ev->kind = EV_CAN;
        ev->frame.id = (uint16_t)id;
        while (1) {
            p = next_token(p, b, sizeof(b));
            if (!*b) {
                break;
            }
            if (ev->frame.dlc >= 8) {
                fail(err, err_len, "CAN frame has more than 8 data bytes", NULL);
                return false;
            }
            unsigned long byte = strtoul(b, &end, 16);
            if (*end != '\0' || byte > 0xFF) {
                fail(err, err_len, "bad CAN data byte", b);
                return false;
            }
            ev->frame.data[ev->frame.dlc++] = (uint8_t)byte;
        }
        snprintf(ev->text, sizeof(ev->text), "can 0x%03X dlc=%u", (unsigned)id,
                 (unsigned)ev->frame.dlc);
        return true;
    }
    if (!strcmp(cmd, "can_cmd")) {
        return build_can_cmd(ev, p, err, err_len);
    }
    if (!strcmp(cmd, "fault") || !strcmp(cmd, "clear")) {
        p = next_token(p, a, sizeof(a));
        p = next_token(p, b, sizeof(b));
        next_token(p, c, sizeof(c));
        if (!sim_fault_parse(a, &ev->fault.kind)) {
            fail(err, err_len, "unknown fault", a);
            return false;
        }
        if ((*b && !parse_num(b, &ev->fault.a)) || (*c && !parse_num(c, &ev->fault.b))) {
            fail(err, err_len, "bad fault argument for", a);
            return false;
        }
        ev->kind = !strcmp(cmd, "fault") ? EV_FAULT : EV_CLEAR;
        snprintf(ev->text, sizeof(ev->text), "%s %s", cmd, a);
        return true;
    }
    if (!strcmp(cmd, "mark")) {
        next_token(p, a, sizeof(a));
        if (!*a) {
            fail(err, err_len, "mark needs a label", NULL);
            return false;
        }
        ev->kind = EV_MARK;
        snprintf(ev->text, sizeof(ev->text), "%s", a);
        return true;
    }
    if (!strcmp(cmd, "end")) {
        ev->kind = EV_END;
        snprintf(ev->text, sizeof(ev->text), "end");
        return true;
    }
    fail(err, err_len, "unknown command", cmd);
    return false;
}

static int cmp_events(const void *x, const void *y)
{
    const scenario_event_t *a = (const scenario_event_t *)x;
    const scenario_event_t *b = (const scenario_event_t *)y;
    if (a->time_us != b->time_us) {
        return a->time_us < b->time_us ? -1 : 1;
    }
    return a->line - b->line; /* stable: keep file order at equal times */
}

bool scenario_parse_string(sim_scenario_t *sc, const char *text, char *err, size_t err_len)
{
    const char *p = text;
    int lineno = 0;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t len = nl ? (size_t)(nl - p) : strlen(p);
        char line[256];
        if (len >= sizeof(line)) {
            len = sizeof(line) - 1;
        }
        memcpy(line, p, len);
        line[len] = '\0';
        p = nl ? nl + 1 : p + len;
        lineno++;

        /* Comments start at '#' unless inside a quoted uart string. */
        bool quoted = false;
        for (char *q = line; *q; q++) {
            if (*q == '"') {
                quoted = !quoted;
            } else if (*q == '#' && !quoted) {
                *q = '\0';
                break;
            }
        }
        if (!*skip_ws(line)) {
            continue;
        }
        if (sc->count >= SCENARIO_MAX_EVENTS) {
            fail(err, err_len, "too many scenario events", NULL);
            return false;
        }
        scenario_event_t *ev = &sc->events[sc->count];
        char inner[200];
        if (!scenario_parse_line(ev, line, inner, sizeof(inner))) {
            if (err && err_len) {
                snprintf(err, err_len, "line %d: %s", lineno, inner);
            }
            return false;
        }
        ev->line = lineno;
        if (ev->kind == EV_END) {
            if (!sc->has_end || ev->time_us < sc->end_us) {
                sc->end_us = ev->time_us;
            }
            sc->has_end = true;
        }
        sc->count++;
    }
    qsort(sc->events, sc->count, sizeof(sc->events[0]), cmp_events);
    sc->next = 0;
    return true;
}

bool scenario_load_file(sim_scenario_t *sc, const char *path, char *err, size_t err_len)
{
    char *text = sim_read_text_file(path, NULL);
    if (!text) {
        fail(err, err_len, "cannot read scenario", path);
        return false;
    }
    bool ok = scenario_parse_string(sc, text, err, err_len);
    free(text);
    return ok;
}

static void apply(const scenario_event_t *ev, sim_board_t *b)
{
    switch (ev->kind) {
    case EV_ENABLE: board_set_enable(b, ev->value != 0.0); break;
    case EV_ESTOP: board_set_estop(b, ev->value != 0.0); break;
    case EV_LOAD: plant_set_load(&b->plant, ev->value); break;
    case EV_VIN: plant_set_vin(&b->plant, ev->value); break;
    case EV_UART: board_uart_send(b, ev->text); break;
    case EV_CAN: board_bench_can_send(b, ev->frame.id, ev->frame.data, ev->frame.dlc); break;
    case EV_FAULT: board_inject_fault(b, &ev->fault); break;
    case EV_CLEAR: board_clear_fault(b, &ev->fault); break;
    case EV_MARK:
    case EV_END: break;
    }
}

size_t scenario_apply_due(sim_scenario_t *sc, sim_board_t *b, scenario_event_cb cb, void *ctx)
{
    size_t applied = 0;
    sim_time_us now = board_now(b);
    while (sc->next < sc->count && sc->events[sc->next].time_us <= now) {
        const scenario_event_t *ev = &sc->events[sc->next++];
        apply(ev, b);
        if (cb) {
            cb(ctx, ev);
        }
        applied++;
    }
    return applied;
}

const char *scenario_event_name(scenario_event_kind_t k)
{
    switch (k) {
    case EV_ENABLE: return "enable";
    case EV_ESTOP: return "estop";
    case EV_LOAD: return "load";
    case EV_VIN: return "vin";
    case EV_UART: return "uart";
    case EV_CAN: return "can";
    case EV_FAULT: return "fault";
    case EV_CLEAR: return "clear";
    case EV_MARK: return "mark";
    case EV_END: return "end";
    }
    return "unknown";
}
