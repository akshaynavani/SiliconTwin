#include "sim/config.h"

#include <ctype.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { T_DOUBLE, T_FLOAT, T_U64, T_U32, T_U16, T_U8, T_BOOL, T_STR } field_type_t;

typedef struct {
    const char *section;
    const char *key;
    field_type_t type;
    size_t offset;
    double scale; /* multiply parsed numbers (unit conversion), 0 = 1 */
    size_t str_len;
} field_t;

#define F(sec, k, t, member) {sec, k, t, offsetof(sim_config_t, member), 0.0, 0}
#define FS(sec, k, t, member, s) {sec, k, t, offsetof(sim_config_t, member), s, 0}

static const field_t k_fields[] = {
    {"sim", "name", T_STR, offsetof(sim_config_t, name), 0.0, sizeof(((sim_config_t *)0)->name)},
    F("sim", "duration_ms", T_U32, duration_ms),
    F("sim", "seed", T_U64, board.seed),
    F("sim", "step_us", T_U32, board.step_us),
    F("sim", "plant_substeps", T_U32, board.plant_substeps),
    F("sim", "trace_interval_us", T_U32, trace_interval_us),

    F("plant", "vin", T_DOUBLE, board.plant.vin),
    FS("plant", "inductance_uh", T_DOUBLE, board.plant.inductance, 1e-6),
    FS("plant", "capacitance_uf", T_DOUBLE, board.plant.capacitance, 1e-6),
    F("plant", "r_inductor", T_DOUBLE, board.plant.r_inductor),
    F("plant", "r_load", T_DOUBLE, board.plant.r_load),
    F("plant", "t_ambient", T_DOUBLE, board.plant.t_ambient),
    F("plant", "r_thermal", T_DOUBLE, board.plant.r_thermal),
    F("plant", "tau_thermal", T_DOUBLE, board.plant.tau_thermal),
    F("plant", "k_switching", T_DOUBLE, board.plant.k_switching),

    F("bench", "enable", T_BOOL, enable_at_start),
    F("bench", "uart_baud", T_U32, board.uart_baud),
    F("bench", "can_bitrate", T_U32, board.can_bitrate),
    F("bench", "pwm_freq_hz", T_U32, board.pwm_freq_hz),
    F("bench", "adc_noise_lsb", T_DOUBLE, board.adc_noise_lsb),

    F("firmware", "flash", T_BOOL, flash_params),
    F("firmware", "corrupt_flash", T_BOOL, corrupt_flash),
    F("firmware", "vref", T_FLOAT, fw.vref),
    F("firmware", "kp_v", T_FLOAT, fw.kp_v),
    F("firmware", "ki_v", T_FLOAT, fw.ki_v),
    F("firmware", "kp_i", T_FLOAT, fw.kp_i),
    F("firmware", "current_limit", T_FLOAT, fw.current_limit),
    F("firmware", "soft_start_ms", T_U32, fw.soft_start_ms),
    F("firmware", "ocp_trip", T_FLOAT, fw.ocp_trip),
    F("firmware", "ovp_trip", T_FLOAT, fw.ovp_trip),
    F("firmware", "uvp_fraction", T_FLOAT, fw.uvp_fraction),
    F("firmware", "vin_min", T_FLOAT, fw.vin_min),
    F("firmware", "vin_max", T_FLOAT, fw.vin_max),
    F("firmware", "otp_trip", T_FLOAT, fw.otp_trip),
    F("firmware", "otp_clear", T_FLOAT, fw.otp_clear),
    F("firmware", "sensor_tolerance", T_FLOAT, fw.sensor_tolerance),
    F("firmware", "retry_delay_ms", T_U32, fw.retry_delay_ms),
    F("firmware", "max_retries", T_U8, fw.max_retries),
    F("firmware", "can_node_id", T_U8, fw.can_node_id),
    F("firmware", "telemetry_period_ms", T_U16, fw.telemetry_period_ms),
    F("firmware", "wdt_timeout_ms", T_U32, fw.wdt_timeout_ms),
};

void sim_config_defaults(sim_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    snprintf(cfg->name, sizeof(cfg->name), "default");
    board_default_config(&cfg->board);
    fw_params_defaults(&cfg->fw);
    cfg->flash_params = true;
    cfg->corrupt_flash = false;
    cfg->enable_at_start = true;
    cfg->duration_ms = 200;
    cfg->trace_interval_us = 100;
}

static void set_err(char *err, size_t len, const char *fmt, const char *a, const char *b)
{
    if (err && len) {
        snprintf(err, len, fmt, a, b);
    }
}

static bool parse_bool(const char *v, bool *out)
{
    if (!strcmp(v, "1") || !strcmp(v, "true") || !strcmp(v, "yes") || !strcmp(v, "on")) {
        *out = true;
        return true;
    }
    if (!strcmp(v, "0") || !strcmp(v, "false") || !strcmp(v, "no") || !strcmp(v, "off")) {
        *out = false;
        return true;
    }
    return false;
}

bool sim_config_set(sim_config_t *cfg, const char *section, const char *key, const char *value,
                    char *err, size_t err_len)
{
    for (size_t i = 0; i < sizeof(k_fields) / sizeof(k_fields[0]); i++) {
        const field_t *f = &k_fields[i];
        if (strcmp(f->section, section) != 0 || strcmp(f->key, key) != 0) {
            continue;
        }
        void *dst = (char *)cfg + f->offset;
        if (f->type == T_STR) {
            snprintf((char *)dst, f->str_len, "%s", value);
            return true;
        }
        if (f->type == T_BOOL) {
            bool b;
            if (!parse_bool(value, &b)) {
                set_err(err, err_len, "%s: expected boolean, got '%s'", key, value);
                return false;
            }
            *(bool *)dst = b;
            return true;
        }
        char *end = NULL;
        errno = 0;
        double d = strtod(value, &end);
        if (end == value || *end != '\0' || errno == ERANGE) {
            set_err(err, err_len, "%s: expected number, got '%s'", key, value);
            return false;
        }
        if (f->scale != 0.0) {
            d *= f->scale;
        }
        bool is_int = f->type != T_DOUBLE && f->type != T_FLOAT;
        if (is_int && d < 0.0) {
            set_err(err, err_len, "%s: must not be negative ('%s')", key, value);
            return false;
        }
        switch (f->type) {
        case T_DOUBLE: *(double *)dst = d; break;
        case T_FLOAT: *(float *)dst = (float)d; break;
        case T_U64: *(uint64_t *)dst = (uint64_t)d; break;
        case T_U32: *(uint32_t *)dst = (uint32_t)d; break;
        case T_U16: *(uint16_t *)dst = (uint16_t)d; break;
        case T_U8: *(uint8_t *)dst = (uint8_t)d; break;
        case T_BOOL:
        case T_STR: break;
        }
        return true;
    }
    set_err(err, err_len, "unknown key '%s.%s'", section, key);
    return false;
}

bool sim_config_apply_override(sim_config_t *cfg, const char *assignment, char *err,
                               size_t err_len)
{
    char buf[256];
    snprintf(buf, sizeof(buf), "%s", assignment);
    char *eq = strchr(buf, '=');
    char *dot = strchr(buf, '.');
    if (!eq || !dot || dot > eq) {
        set_err(err, err_len, "override '%s' must look like section.key=value%s", assignment, "");
        return false;
    }
    *eq = '\0';
    *dot = '\0';
    return sim_config_set(cfg, buf, dot + 1, eq + 1, err, err_len);
}

static char *trim(char *s)
{
    while (isspace((unsigned char)*s)) {
        s++;
    }
    char *e = s + strlen(s);
    while (e > s && isspace((unsigned char)e[-1])) {
        *--e = '\0';
    }
    return s;
}

bool sim_config_load_string(sim_config_t *cfg, const char *text, char *err, size_t err_len)
{
    char section[32] = "";
    const char *p = text;
    int lineno = 0;
    while (*p) {
        const char *nl = strchr(p, '\n');
        size_t len = nl ? (size_t)(nl - p) : strlen(p);
        char line[512];
        if (len >= sizeof(line)) {
            len = sizeof(line) - 1;
        }
        memcpy(line, p, len);
        line[len] = '\0';
        p = nl ? nl + 1 : p + len;
        lineno++;

        char *hash = strpbrk(line, "#;");
        if (hash) {
            *hash = '\0';
        }
        char *s = trim(line);
        if (!*s) {
            continue;
        }
        if (*s == '[') {
            char *close = strchr(s, ']');
            if (!close) {
                char ln[16];
                snprintf(ln, sizeof(ln), "%d", lineno);
                set_err(err, err_len, "line %s: unterminated section header%s", ln, "");
                return false;
            }
            *close = '\0';
            snprintf(section, sizeof(section), "%s", trim(s + 1));
            continue;
        }
        char *eq = strchr(s, '=');
        if (!eq) {
            char ln[16];
            snprintf(ln, sizeof(ln), "%d", lineno);
            set_err(err, err_len, "line %s: expected key = value%s", ln, "");
            return false;
        }
        *eq = '\0';
        char *key = trim(s);
        char *value = trim(eq + 1);
        if (!sim_config_set(cfg, section, key, value, err, err_len)) {
            return false;
        }
    }
    return true;
}

char *sim_read_text_file(const char *path, size_t *len_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        return NULL;
    }
    long size = ftell(f);
    if (size < 0) {
        fclose(f);
        return NULL;
    }
    rewind(f);
    char *buf = (char *)malloc((size_t)size + 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }
    size_t n = fread(buf, 1, (size_t)size, f);
    fclose(f);
    buf[n] = '\0';
    if (len_out) {
        *len_out = n;
    }
    return buf;
}

bool sim_config_load_file(sim_config_t *cfg, const char *path, char *err, size_t err_len)
{
    char *text = sim_read_text_file(path, NULL);
    if (!text) {
        set_err(err, err_len, "cannot read config '%s'%s", path, "");
        return false;
    }
    bool ok = sim_config_load_string(cfg, text, err, err_len);
    free(text);
    return ok;
}
