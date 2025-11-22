/*
 * Non-blocking UART command shell.
 *
 * Input bytes are assembled into lines (CR, LF or CRLF terminated). Output is
 * staged in a software ring buffer and drained into the UART hardware FIFO
 * from the main loop, so printing never blocks the control firmware.
 * A line that saw a UART error (overrun / framing) is discarded rather than
 * executed, because a corrupted command must never be acted on.
 */
#ifndef FW_SHELL_H
#define FW_SHELL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FW_SHELL_LINE_MAX 64
#define FW_SHELL_TX_BUF 1024
#define FW_SHELL_MAX_ARGS 6

typedef void (*fw_shell_handler_t)(int argc, char **argv);

typedef struct {
    uint32_t lines;
    uint32_t discarded;
    uint32_t uart_errors;
    uint32_t tx_dropped;
} fw_shell_stats_t;

void fw_shell_init(fw_shell_handler_t handler);
/* Reads available input, executes complete lines and flushes output. */
void fw_shell_poll(void);
void fw_shell_printf(const char *fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;
void fw_shell_flush(void);
const fw_shell_stats_t *fw_shell_stats(void);

/* Splits a line in place on whitespace. Exposed for unit testing. */
int fw_shell_tokenize(char *line, char **argv, int max_args);

#ifdef __cplusplus
}
#endif

#endif /* FW_SHELL_H */
