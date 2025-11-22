#include "fw/shell.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "fw/hal.h"

static fw_shell_handler_t s_handler;
static char s_line[FW_SHELL_LINE_MAX + 1];
static size_t s_line_len;
static bool s_line_bad;
static char s_tx[FW_SHELL_TX_BUF];
static size_t s_tx_head;
static size_t s_tx_count;
static fw_shell_stats_t s_stats;

void fw_shell_init(fw_shell_handler_t handler)
{
    s_handler = handler;
    s_line_len = 0;
    s_line_bad = false;
    s_tx_head = 0;
    s_tx_count = 0;
    memset(&s_stats, 0, sizeof(s_stats));
}

static void tx_push(const char *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        if (s_tx_count >= FW_SHELL_TX_BUF) {
            s_stats.tx_dropped += (uint32_t)(len - i);
            return;
        }
        s_tx[(s_tx_head + s_tx_count) % FW_SHELL_TX_BUF] = data[i];
        s_tx_count++;
    }
}

void fw_shell_printf(const char *fmt, ...)
{
    char buf[160];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    size_t len = (size_t)n < sizeof(buf) ? (size_t)n : sizeof(buf) - 1;
    tx_push(buf, len);
}

void fw_shell_flush(void)
{
    while (s_tx_count > 0) {
        /* Write the contiguous run up to the end of the ring. */
        size_t run = FW_SHELL_TX_BUF - s_tx_head;
        if (run > s_tx_count) {
            run = s_tx_count;
        }
        size_t written = hal_uart_write(&s_tx[s_tx_head], run);
        s_tx_head = (s_tx_head + written) % FW_SHELL_TX_BUF;
        s_tx_count -= written;
        if (written < run) {
            break; /* hardware FIFO full; continue next tick */
        }
    }
}

int fw_shell_tokenize(char *line, char **argv, int max_args)
{
    int argc = 0;
    char *p = line;
    while (*p && argc < max_args) {
        while (*p == ' ' || *p == '\t') {
            p++;
        }
        if (!*p) {
            break;
        }
        argv[argc++] = p;
        while (*p && *p != ' ' && *p != '\t') {
            p++;
        }
        if (*p) {
            *p++ = '\0';
        }
    }
    return argc;
}

static void finish_line(void)
{
    s_line[s_line_len] = '\0';
    if (s_line_bad) {
        s_stats.discarded++;
        fw_shell_printf("ERR line discarded\r\n");
    } else if (s_line_len > 0) {
        char *argv[FW_SHELL_MAX_ARGS];
        int argc = fw_shell_tokenize(s_line, argv, FW_SHELL_MAX_ARGS);
        if (argc > 0 && s_handler) {
            s_stats.lines++;
            s_handler(argc, argv);
        }
    }
    s_line_len = 0;
    s_line_bad = false;
}

void fw_shell_poll(void)
{
    if (hal_uart_take_error()) {
        s_stats.uart_errors++;
        s_line_bad = true;
    }

    uint8_t buf[32];
    size_t n;
    while ((n = hal_uart_read(buf, sizeof(buf))) > 0) {
        for (size_t i = 0; i < n; i++) {
            char c = (char)buf[i];
            if (c == '\r' || c == '\n') {
                if (s_line_len > 0 || s_line_bad) {
                    finish_line();
                }
            } else if (c == 0x08 || c == 0x7F) {
                if (s_line_len > 0) {
                    s_line_len--;
                }
            } else if (s_line_len < FW_SHELL_LINE_MAX) {
                if ((unsigned char)c < 0x20 || (unsigned char)c > 0x7E) {
                    s_line_bad = true; /* non-printable: likely corruption */
                }
                s_line[s_line_len++] = c;
            } else {
                s_line_bad = true; /* overlong line */
            }
        }
    }
    fw_shell_flush();
}

const fw_shell_stats_t *fw_shell_stats(void)
{
    return &s_stats;
}
