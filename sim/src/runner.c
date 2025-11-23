#include "sim/runner.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fw/can_proto.h"
#include "fw/protection.h"

#define SETTLE_BAND 0.02
#define OVERSHOOT_HOLD_MS 30.0

typedef struct {
    sim_board_t *board;
    sim_metrics_t *m;
    fw_state_t prev_state;
    uint8_t prev_fault;
    sim_time_us enable_at;      /* first enable; UINT64_MAX if never */
    bool enabled_seen;
    sim_time_us first_pgood_at;
    bool pgood_seen;
    double startup_vmax;
    double startup_vref;
    double tail[RUN_TAIL_SAMPLES];
    uint32_t tail_count;
    uint32_t tail_head;
    sim_time_us next_tail;
    sim_time_us next_trace;
} run_ctx_t;

static double ms(sim_time_us t)
{
    return (double)t / 1000.0;
}

static void close_window(run_ctx_t *c, sim_time_us now)
{
    sim_metrics_t *m = c->m;
    if (m->n_windows == 0) {
        return;
    }
    run_window_t *w = &m->windows[m->n_windows - 1];
    if (!w->open) {
        return;
    }
    w->open = false;
    w->end_ms = ms(now);
    if (w->in_band) {
        double start_us = w->start_ms * 1000.0;
        w->settle_ms = w->last_out_of_band > (sim_time_us)start_us
                           ? ((double)w->last_out_of_band - start_us) / 1000.0
                           : 0.0;
    } else {
        w->settle_ms = -1.0;
    }
}

static void on_event(void *ctx, const scenario_event_t *ev)
{
    run_ctx_t *c = (run_ctx_t *)ctx;
    sim_metrics_t *m = c->m;
    sim_time_us now = board_now(c->board);
    m->events_applied++;

    if (ev->kind == EV_ENABLE && ev->value != 0.0 && !c->enabled_seen) {
        c->enabled_seen = true;
        c->enable_at = now;
    }
    if (ev->kind == EV_MARK) {
        close_window(c, now);
        if (m->n_windows < RUN_MAX_WINDOWS) {
            run_window_t *w = &m->windows[m->n_windows++];
            memset(w, 0, sizeof(*w));
            snprintf(w->label, sizeof(w->label), "%s", ev->text);
            w->start_ms = ms(now);
            w->vout_min = 1e9;
            w->vout_max = -1e9;
            w->iout_max = -1e9;
            w->last_out_of_band = now;
            w->open = true;
        }
    }
}

static void drain_bench_can(run_ctx_t *c)
{
    sim_metrics_t *m = c->m;
    can_frame_t f;
    while (board_bench_can_receive(c->board, &f)) {
        uint16_t base = (uint16_t)(f.id & 0x780u);
        if (base == FW_CAN_ID_TELEM) {
            m->rx_telemetry++;
        } else if (base == FW_CAN_ID_STATUS) {
            m->rx_status++;
        } else if (base == FW_CAN_ID_FAULT) {
            m->rx_fault++;
        } else if (base == FW_CAN_ID_BOOT) {
            m->rx_boot++;
        } else if (base == FW_CAN_ID_ACK) {
            m->rx_ack++;
            if (m->n_acks < RUN_MAX_ACKS && f.dlc >= 2) {
                run_ack_t *a = &m->acks[m->n_acks++];
                a->t_ms = ms(board_now(c->board));
                a->cmd = f.data[0];
                a->status = f.data[1];
            }
        }
    }
}

static void observe(run_ctx_t *c, FILE *trace, uint32_t trace_interval_us)
{
    sim_board_t *b = c->board;
    sim_metrics_t *m = c->m;
    sim_time_us now = board_now(b);
    const fw_status_t *st = fw_status();
    double v = b->plant.v_out;
    double i = b->plant.i_l;

    if (v < m->vout_min) m->vout_min = v;
    if (v > m->vout_max) m->vout_max = v;
    if (i > m->iout_peak) m->iout_peak = i;
    if (b->plant.temp > m->temp_peak) m->temp_peak = b->plant.temp;

    if (st->state != c->prev_state) {
        if (m->n_transitions < RUN_MAX_TRANSITIONS) {
            run_transition_t *t = &m->transitions[m->n_transitions++];
            t->t_ms = ms(now);
            t->from = c->prev_state;
            t->to = st->state;
        }
        c->prev_state = st->state;
    }
    if (st->active_fault != c->prev_fault) {
        if (st->active_fault != FW_FAULT_NONE && m->n_faults < RUN_MAX_FAULTS) {
            m->faults[m->n_faults].t_ms = ms(now);
            m->faults[m->n_faults].fault = st->active_fault;
            m->n_faults++;
        }
        c->prev_fault = st->active_fault;
    }

    /* Start-up characterisation. */
    if (c->enabled_seen && !c->pgood_seen && st->pgood) {
        c->pgood_seen = true;
        c->first_pgood_at = now;
        m->startup_ms = ms(now - c->enable_at);
        c->startup_vref = st->vref_target;
    }
    if (c->enabled_seen && (!c->pgood_seen || ms(now - c->first_pgood_at) <= OVERSHOOT_HOLD_MS)) {
        if (v > c->startup_vmax) {
            c->startup_vmax = v;
        }
    }

    /* Measurement window. */
    if (m->n_windows > 0 && m->windows[m->n_windows - 1].open) {
        run_window_t *w = &m->windows[m->n_windows - 1];
        if (v < w->vout_min) w->vout_min = v;
        if (v > w->vout_max) w->vout_max = v;
        if (i > w->iout_max) w->iout_max = i;
        double band = SETTLE_BAND * st->vref_target;
        bool in = fabs(v - st->vref_target) <= band && st->state == FW_STATE_REGULATING;
        if (!in) {
            w->last_out_of_band = now;
        }
        w->in_band = in;
    }

    if (now >= c->next_tail) {
        c->next_tail = now + 100;
        c->tail[c->tail_head] = v;
        c->tail_head = (c->tail_head + 1u) % RUN_TAIL_SAMPLES;
        if (c->tail_count < RUN_TAIL_SAMPLES) {
            c->tail_count++;
        }
    }

    if (trace && now >= c->next_trace) {
        c->next_trace = now + trace_interval_us;
        fprintf(trace, "%.3f,%.4f,%.4f,%.3f,%.2f,%.4f,%.4f,%s,%d,%s\n", ms(now), v, i,
                b->plant.p.vin, b->plant.temp, pwm_effective_duty(&b->mcu.pwm), st->vref_now,
                fw_state_name(st->state), st->pgood ? 1 : 0, fw_fault_name(st->active_fault));
    }

    uint16_t tec = b->can.nodes[b->mcu_node].tec;
    if (tec > m->can_max_tec) {
        m->can_max_tec = tec;
    }
    drain_bench_can(c);
}

static void finalize(run_ctx_t *c)
{
    sim_board_t *b = c->board;
    sim_metrics_t *m = c->m;
    sim_time_us now = board_now(b);
    const fw_status_t *st = fw_status();

    close_window(c, now);
    m->sim_time_ms = ms(now);
    m->vout_final = b->plant.v_out;
    m->iout_final = b->plant.i_l;
    m->vin_final = b->plant.p.vin;
    m->vref_final = st->vref_target;
    m->energy_j = b->plant.energy_out_j;
    m->pgood_final = st->pgood;
    m->final_state = st->state;
    m->final_fault = st->active_fault;
    m->fault_history = st->fault_history;
    m->retries_final = st->retries;
    m->params_from_flash = st->params_from_flash;

    if (c->pgood_seen && c->startup_vref > 0.0) {
        double os = (c->startup_vmax - c->startup_vref) / c->startup_vref * 100.0;
        m->overshoot_pct = os > 0.0 ? os : 0.0;
    }

    if (c->tail_count > 0) {
        double sum = 0.0, lo = 1e9, hi = -1e9;
        for (uint32_t k = 0; k < c->tail_count; k++) {
            double v = c->tail[k];
            sum += fabs(v - m->vref_final);
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        m->steady_state_error = sum / (double)c->tail_count;
        m->ripple_pp = hi - lo;
    }

    const mcu_t *mcu = &b->mcu;
    uint32_t logged = mcu->reset_count < MCU_RESET_LOG_SIZE ? mcu->reset_count : MCU_RESET_LOG_SIZE;
    for (uint32_t k = 0; k < logged; k++) {
        if (mcu->reset_log[k].cause == HAL_RESET_POWER_ON || m->n_resets >= RUN_MAX_RESETS) {
            continue;
        }
        m->resets[m->n_resets].t_ms = ms(mcu->reset_log[k].timestamp_us);
        m->resets[m->n_resets].cause = mcu->reset_log[k].cause;
        m->n_resets++;
    }

    m->wdt_expirations = mcu->wdt.expirations;
    m->wdt_kicks = mcu->wdt.kicks;
    m->wdt_max_kick_gap_ms = ms(mcu->wdt.max_kick_interval);

    const can_node_t *node = &b->can.nodes[b->mcu_node];
    m->can_frames_ok = b->can.frames_ok;
    m->can_error_frames = b->can.error_frames;
    m->can_bus_off_events = node->stats.bus_off_events;
    m->can_final_state = node->state;

    size_t len = 0;
    const char *out = uart_host_output(&mcu->uart, &len);
    m->uart_output = (char *)malloc(len + 1);
    if (m->uart_output) {
        memcpy(m->uart_output, out, len);
        m->uart_output[len] = '\0';
    }
    m->uart_stats = mcu->uart.stats;
}

bool sim_run(const sim_config_t *cfg, sim_scenario_t *sc, sim_metrics_t *m, FILE *trace,
             char *err, size_t err_len)
{
    memset(m, 0, sizeof(*m));
    snprintf(m->name, sizeof(m->name), "%s", cfg->name);
    m->vout_min = 1e9;
    m->vout_max = -1e9;
    m->startup_ms = -1.0;

    sim_board_t *b = (sim_board_t *)calloc(1, sizeof(*b));
    if (!b) {
        snprintf(err, err_len, "out of memory");
        return false;
    }
    mcu_firmware_t fw = {fw_init, fw_main_tick};
    board_init(b, &cfg->board, &fw);

    if (cfg->flash_params) {
        fw_params_t image = cfg->fw;
        fw_params_seal(&image);
        if (cfg->corrupt_flash) {
            image.crc ^= 0xA5A5A5A5u;
        }
        mcu_flash_program(&b->mcu, FW_PARAMS_FLASH_OFFSET, &image, sizeof(image));
    }

    run_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    ctx.board = b;
    ctx.m = m;

    if (cfg->enable_at_start) {
        board_set_enable(b, true);
        ctx.enabled_seen = true;
        ctx.enable_at = 0;
    }
    board_power_on(b);
    ctx.prev_state = fw_status()->state;
    ctx.prev_fault = fw_status()->active_fault;

    if (trace) {
        fprintf(trace, "t_ms,vout,iout,vin,temp,duty,vref,state,pgood,fault\n");
    }

    sim_time_us end = SIM_MS(cfg->duration_ms);
    if (sc && sc->has_end) {
        end = sc->end_us;
    }
    if (sc) {
        sc->next = 0;
    }

    while (board_now(b) < end) {
        if (sc) {
            scenario_apply_due(sc, b, on_event, &ctx);
        }
        board_step(b);
        m->steps++;
        observe(&ctx, trace, cfg->trace_interval_us ? cfg->trace_interval_us : 100u);
    }
    finalize(&ctx);
    free(b);
    return true;
}

void sim_metrics_free(sim_metrics_t *m)
{
    free(m->uart_output);
    m->uart_output = NULL;
}

/* ----------------------------------------------------------- JSON output */

static void json_str(FILE *o, const char *s)
{
    fputc('"', o);
    for (; s && *s; s++) {
        unsigned char ch = (unsigned char)*s;
        switch (ch) {
        case '"': fputs("\\\"", o); break;
        case '\\': fputs("\\\\", o); break;
        case '\n': fputs("\\n", o); break;
        case '\r': fputs("\\r", o); break;
        case '\t': fputs("\\t", o); break;
        default:
            if (ch < 0x20 || ch >= 0x7F) {
                fprintf(o, "\\u%04x", ch);
            } else {
                fputc((int)ch, o);
            }
        }
    }
    fputc('"', o);
}

static void json_num(FILE *o, double v)
{
    if (isnan(v) || isinf(v)) {
        fputs("null", o);
    } else {
        fprintf(o, "%.6g", v);
    }
}

static void json_fault_list(FILE *o, uint8_t bits)
{
    fputc('[', o);
    bool first = true;
    for (unsigned bit = 1; bit <= 0x80u; bit <<= 1) {
        if (bits & bit) {
            if (!first) {
                fputc(',', o);
            }
            json_str(o, fw_fault_name((uint8_t)bit));
            first = false;
        }
    }
    fputc(']', o);
}

void sim_metrics_write_json(const sim_metrics_t *m, FILE *o)
{
    fputs("{\n  \"name\": ", o);
    json_str(o, m->name);
    fputs(",\n  \"sim_time_ms\": ", o); json_num(o, m->sim_time_ms);
    fprintf(o, ",\n  \"steps\": %llu", (unsigned long long)m->steps);

    fputs(",\n  \"regulation\": {", o);
    fputs("\"vout_final\": ", o); json_num(o, m->vout_final);
    fputs(", \"iout_final\": ", o); json_num(o, m->iout_final);
    fputs(", \"vin_final\": ", o); json_num(o, m->vin_final);
    fputs(", \"vref_final\": ", o); json_num(o, m->vref_final);
    fputs(", \"vout_min\": ", o); json_num(o, m->vout_min);
    fputs(", \"vout_max\": ", o); json_num(o, m->vout_max);
    fputs(", \"iout_peak\": ", o); json_num(o, m->iout_peak);
    fputs(", \"temp_peak\": ", o); json_num(o, m->temp_peak);
    fputs(", \"energy_j\": ", o); json_num(o, m->energy_j);
    fputs(", \"startup_ms\": ", o); json_num(o, m->startup_ms);
    fputs(", \"overshoot_pct\": ", o); json_num(o, m->overshoot_pct);
    fputs(", \"steady_state_error\": ", o); json_num(o, m->steady_state_error);
    fputs(", \"ripple_pp\": ", o); json_num(o, m->ripple_pp);
    fprintf(o, ", \"pgood_final\": %s}", m->pgood_final ? "true" : "false");

    fputs(",\n  \"firmware\": {\"final_state\": ", o);
    json_str(o, fw_state_name(m->final_state));
    fputs(", \"final_fault\": ", o);
    json_str(o, fw_fault_name(m->final_fault));
    fputs(", \"fault_history\": ", o);
    json_fault_list(o, m->fault_history);
    fprintf(o, ", \"retries\": %u, \"params_from_flash\": %s}", (unsigned)m->retries_final,
            m->params_from_flash ? "true" : "false");

    fputs(",\n  \"faults\": [", o);
    for (uint32_t k = 0; k < m->n_faults; k++) {
        fputs(k ? ", " : "", o);
        fputs("{\"t_ms\": ", o); json_num(o, m->faults[k].t_ms);
        fputs(", \"fault\": ", o); json_str(o, fw_fault_name(m->faults[k].fault));
        fputc('}', o);
    }
    fputs("]", o);

    fputs(",\n  \"transitions\": [", o);
    for (uint32_t k = 0; k < m->n_transitions; k++) {
        fputs(k ? ", " : "", o);
        fputs("{\"t_ms\": ", o); json_num(o, m->transitions[k].t_ms);
        fputs(", \"from\": ", o); json_str(o, fw_state_name(m->transitions[k].from));
        fputs(", \"to\": ", o); json_str(o, fw_state_name(m->transitions[k].to));
        fputc('}', o);
    }
    fputs("]", o);

    fputs(",\n  \"windows\": [", o);
    for (uint32_t k = 0; k < m->n_windows; k++) {
        const run_window_t *w = &m->windows[k];
        fputs(k ? ", " : "", o);
        fputs("{\"label\": ", o); json_str(o, w->label);
        fputs(", \"start_ms\": ", o); json_num(o, w->start_ms);
        fputs(", \"end_ms\": ", o); json_num(o, w->end_ms);
        fputs(", \"vout_min\": ", o); json_num(o, w->vout_min);
        fputs(", \"vout_max\": ", o); json_num(o, w->vout_max);
        fputs(", \"iout_max\": ", o); json_num(o, w->iout_max);
        fputs(", \"settle_ms\": ", o); json_num(o, w->settle_ms);
        fputc('}', o);
    }
    fputs("]", o);

    fputs(",\n  \"resets\": [", o);
    for (uint32_t k = 0; k < m->n_resets; k++) {
        fputs(k ? ", " : "", o);
        fputs("{\"t_ms\": ", o); json_num(o, m->resets[k].t_ms);
        fputs(", \"cause\": ", o); json_str(o, mcu_reset_cause_name(m->resets[k].cause));
        fputc('}', o);
    }
    fputs("]", o);

    fprintf(o, ",\n  \"watchdog\": {\"expirations\": %u, \"kicks\": %u, \"max_kick_gap_ms\": ",
            (unsigned)m->wdt_expirations, (unsigned)m->wdt_kicks);
    json_num(o, m->wdt_max_kick_gap_ms);
    fputc('}', o);

    fprintf(o,
            ",\n  \"can\": {\"frames_ok\": %u, \"error_frames\": %u, \"bus_off_events\": %u, "
            "\"max_tec\": %u, \"final_state\": ",
            (unsigned)m->can_frames_ok, (unsigned)m->can_error_frames,
            (unsigned)m->can_bus_off_events, (unsigned)m->can_max_tec);
    json_str(o, can_state_name(m->can_final_state));
    fprintf(o,
            ", \"rx\": {\"telemetry\": %u, \"status\": %u, \"fault\": %u, \"boot\": %u, "
            "\"ack\": %u}, \"acks\": [",
            (unsigned)m->rx_telemetry, (unsigned)m->rx_status, (unsigned)m->rx_fault,
            (unsigned)m->rx_boot, (unsigned)m->rx_ack);
    for (uint32_t k = 0; k < m->n_acks; k++) {
        fputs(k ? ", " : "", o);
        fputs("{\"t_ms\": ", o); json_num(o, m->acks[k].t_ms);
        fprintf(o, ", \"cmd\": %u, \"status\": %u}", (unsigned)m->acks[k].cmd,
                (unsigned)m->acks[k].status);
    }
    fputs("]}", o);

    fprintf(o,
            ",\n  \"uart\": {\"tx_bytes\": %u, \"rx_bytes\": %u, \"overruns\": %u, "
            "\"framing_errors\": %u, \"output\": ",
            (unsigned)m->uart_stats.tx_bytes, (unsigned)m->uart_stats.rx_bytes,
            (unsigned)m->uart_stats.overruns, (unsigned)m->uart_stats.framing_errors);
    json_str(o, m->uart_output ? m->uart_output : "");
    fputc('}', o);

    fprintf(o, ",\n  \"events_applied\": %u\n}\n", (unsigned)m->events_applied);
}
