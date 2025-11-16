/*
 * Hardware Abstraction Layer used by the SiliconTwin firmware.
 *
 * This is the only interface the firmware is allowed to use to touch
 * "hardware". The simulator provides one implementation (sim/src/hal_sim.c);
 * a port to a real MCU would provide another, leaving firmware untouched.
 *
 * Board definition: a synchronous buck converter controller.
 *   - PWM drives the high-side switch; duty in [0, 1].
 *   - ADC channels sample output voltage, inductor current, input voltage
 *     and heatsink temperature, synchronised to the PWM period.
 *   - GPIO: enable input, active-low e-stop input, fault LED, power-good
 *     output and a heartbeat LED.
 *   - UART debug/command shell, CAN telemetry/command interface.
 */
#ifndef FW_HAL_H
#define FW_HAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- board pins */
#define HAL_PIN_ENABLE     0u /* input, pull-down: high = run */
#define HAL_PIN_ESTOP_N    1u /* input, pull-up, falling edge IRQ: emergency stop */
#define HAL_PIN_FAULT_LED  2u /* output */
#define HAL_PIN_PGOOD      3u /* output: output voltage in regulation */
#define HAL_PIN_HEARTBEAT  4u /* output: toggles from the main loop */

/* ----------------------------------------------------------------- ADC setup */
#define HAL_ADC_BITS       12u
#define HAL_ADC_MAX_CODE   4095u

typedef enum {
    HAL_ADC_VOUT = 0,
    HAL_ADC_IOUT = 1,
    HAL_ADC_VIN = 2,
    HAL_ADC_TEMP = 3,
    HAL_ADC_NUM_CHANNELS
} hal_adc_channel_t;

/* Full-scale engineering value of each channel (code 4095). */
#define HAL_ADC_VOUT_FULL_SCALE  30.0f  /* volts   */
#define HAL_ADC_IOUT_FULL_SCALE  25.0f  /* amperes */
#define HAL_ADC_VIN_FULL_SCALE   100.0f /* volts   */
#define HAL_ADC_TEMP_FULL_SCALE  200.0f /* deg C   */

/* --------------------------------------------------------------------- types */
typedef enum {
    HAL_EDGE_NONE = 0,
    HAL_EDGE_RISING = 1,
    HAL_EDGE_FALLING = 2,
    HAL_EDGE_BOTH = 3
} hal_edge_t;

typedef enum {
    HAL_RESET_POWER_ON = 0,
    HAL_RESET_WATCHDOG = 1,
    HAL_RESET_SOFTWARE = 2,
    HAL_RESET_BROWNOUT = 3
} hal_reset_cause_t;

typedef enum {
    HAL_CAN_ERROR_ACTIVE = 0,
    HAL_CAN_ERROR_PASSIVE = 1,
    HAL_CAN_BUS_OFF = 2
} hal_can_state_t;

typedef struct {
    uint16_t id;     /* 11-bit standard identifier */
    uint8_t dlc;     /* 0..8 */
    uint8_t data[8];
} hal_can_frame_t;

typedef void (*hal_isr_t)(void);
typedef void (*hal_gpio_isr_t)(uint8_t pin, bool level);

/* --------------------------------------------------------------------- time */
uint32_t hal_millis(void);
uint64_t hal_micros(void);

/* --------------------------------------------------------------------- GPIO */
void hal_gpio_config_output(uint8_t pin, bool initial);
void hal_gpio_config_input(uint8_t pin, bool pull_up, bool pull_down);
void hal_gpio_write(uint8_t pin, bool level);
void hal_gpio_toggle(uint8_t pin);
bool hal_gpio_read(uint8_t pin);
void hal_gpio_enable_irq(uint8_t pin, hal_edge_t edge);
void hal_gpio_set_isr(hal_gpio_isr_t isr);

/* ------------------------------------------------------------- ADC and PWM */
uint16_t hal_adc_read(hal_adc_channel_t ch);
void hal_pwm_enable(bool enabled);
void hal_pwm_set_duty(float duty);
/* Registers the PWM-synchronous control interrupt (ADC end-of-conversion). */
void hal_set_control_isr(hal_isr_t isr, uint32_t rate_hz);

/* --------------------------------------------------------------------- UART */
void hal_uart_init(void);
size_t hal_uart_write(const void *data, size_t len);
size_t hal_uart_read(void *data, size_t max);
bool hal_uart_take_error(void); /* overrun or framing error since last call */

/* ---------------------------------------------------------------------- CAN */
void hal_can_init(uint32_t bitrate);
bool hal_can_set_filter(uint8_t index, uint16_t id, uint16_t mask);
bool hal_can_send(const hal_can_frame_t *frame);
bool hal_can_receive(hal_can_frame_t *frame);
hal_can_state_t hal_can_state(void);
void hal_can_recover(void); /* request bus-off recovery */

/* ----------------------------------------------------- watchdog and reset */
void hal_wdt_start(uint32_t timeout_ms);
void hal_wdt_kick(void);
hal_reset_cause_t hal_reset_cause(void);
void hal_system_reset(void);

/* -------------------------------------------------------- memory regions */
/* RAM that survives warm resets (watchdog/software), cleared on power-on. */
void *hal_retained_ram(size_t *size);
/* Read from the parameter flash page. Returns bytes read. */
size_t hal_flash_read(uint32_t offset, void *dst, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FW_HAL_H */
