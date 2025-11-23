/*
 * silicontwin - command line front end for the firmware simulator.
 *
 *   silicontwin [--config FILE] [--scenario FILE] [--set section.key=value]...
 *               [--duration-ms N] [--seed N] [--json FILE|-] [--trace FILE]
 *               [--quiet]
 *
 * Exit codes: 0 run completed, 2 usage or input error.
 * Pass/fail judgement is left to the caller (the Python harness), which
 * evaluates expectations against the JSON metrics.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "sim/config.h"
#include "sim/runner.h"
#include "sim/scenario.h"

#define MAX_OVERRIDES 64

static void usage(FILE *o)
{
    fprintf(o,
            "usage: silicontwin [options]\n"
            "  --config FILE        INI run configuration\n"
            "  --scenario FILE      timed stimulus / fault-injection script\n"
            "  --set S.K=V          override a config key (repeatable)\n"
            "  --duration-ms N      run length (a scenario 'end' event wins)\n"
            "  --seed N             PRNG seed for noise and fault injection\n"
            "  --json FILE|-        write metrics as JSON (default: stdout)\n"
            "  --trace FILE         write a CSV waveform trace\n"
            "  --quiet              suppress the human-readable summary on stderr\n"
            "  --help               show this help\n");
}

static const char *need_arg(int argc, char **argv, int *i)
{
    if (*i + 1 >= argc) {
        fprintf(stderr, "error: %s needs an argument\n", argv[*i]);
        exit(2);
    }
    return argv[++(*i)];
}

int main(int argc, char **argv)
{
    const char *config_path = NULL;
    const char *scenario_path = NULL;
    const char *json_path = "-";
    const char *trace_path = NULL;
    const char *overrides[MAX_OVERRIDES];
    int n_overrides = 0;
    const char *duration = NULL;
    const char *seed = NULL;
    int quiet = 0;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        if (!strcmp(a, "--config")) {
            config_path = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--scenario")) {
            scenario_path = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--set")) {
            if (n_overrides >= MAX_OVERRIDES) {
                fprintf(stderr, "error: too many --set options\n");
                return 2;
            }
            overrides[n_overrides++] = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--duration-ms")) {
            duration = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--seed")) {
            seed = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--json")) {
            json_path = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--trace")) {
            trace_path = need_arg(argc, argv, &i);
        } else if (!strcmp(a, "--quiet")) {
            quiet = 1;
        } else if (!strcmp(a, "--help") || !strcmp(a, "-h")) {
            usage(stdout);
            return 0;
        } else {
            fprintf(stderr, "error: unknown option '%s'\n", a);
            usage(stderr);
            return 2;
        }
    }

    char err[256] = "";
    sim_config_t cfg;
    sim_config_defaults(&cfg);
    if (config_path && !sim_config_load_file(&cfg, config_path, err, sizeof(err))) {
        fprintf(stderr, "config error: %s\n", err);
        return 2;
    }
    for (int k = 0; k < n_overrides; k++) {
        if (!sim_config_apply_override(&cfg, overrides[k], err, sizeof(err))) {
            fprintf(stderr, "override error: %s\n", err);
            return 2;
        }
    }
    if (duration && !sim_config_set(&cfg, "sim", "duration_ms", duration, err, sizeof(err))) {
        fprintf(stderr, "error: %s\n", err);
        return 2;
    }
    if (seed && !sim_config_set(&cfg, "sim", "seed", seed, err, sizeof(err))) {
        fprintf(stderr, "error: %s\n", err);
        return 2;
    }

    sim_scenario_t *sc = NULL;
    if (scenario_path) {
        sc = (sim_scenario_t *)malloc(sizeof(*sc));
        if (!sc) {
            fprintf(stderr, "error: out of memory\n");
            return 2;
        }
        scenario_init(sc);
        if (!scenario_load_file(sc, scenario_path, err, sizeof(err))) {
            fprintf(stderr, "scenario error: %s\n", err);
            free(sc);
            return 2;
        }
    }

    FILE *trace = NULL;
    if (trace_path) {
        trace = fopen(trace_path, "w");
        if (!trace) {
            fprintf(stderr, "error: cannot open trace file '%s'\n", trace_path);
            free(sc);
            return 2;
        }
    }

    sim_metrics_t *m = (sim_metrics_t *)malloc(sizeof(*m));
    if (!m || !sim_run(&cfg, sc, m, trace, err, sizeof(err))) {
        fprintf(stderr, "run error: %s\n", m ? err : "out of memory");
        free(m);
        free(sc);
        if (trace) {
            fclose(trace);
        }
        return 2;
    }
    if (trace) {
        fclose(trace);
    }

    FILE *out = stdout;
    if (strcmp(json_path, "-") != 0) {
        out = fopen(json_path, "w");
        if (!out) {
            fprintf(stderr, "error: cannot open '%s'\n", json_path);
            sim_metrics_free(m);
            free(m);
            free(sc);
            return 2;
        }
    }
    sim_metrics_write_json(m, out);
    if (out != stdout) {
        fclose(out);
    }

    if (!quiet) {
        fprintf(stderr,
                "[%s] %.1f ms simulated: state=%s vout=%.3f V (ref %.3f) startup=%.2f ms "
                "faults=%u resets=%u\n",
                m->name, m->sim_time_ms, fw_state_name(m->final_state), m->vout_final,
                m->vref_final, m->startup_ms, (unsigned)m->n_faults, (unsigned)m->n_resets);
    }

    sim_metrics_free(m);
    free(m);
    free(sc);
    return 0;
}
