#include "fw/firmware.h"

#include <stdlib.h>
#include <string.h>

#include "fw/can_proto.h"
#include "fw/protection.h"
#include "fw/retained.h"
#include "fw/shell.h"

#define CONTROL_TS (1.0f / (float)FW_CONTROL_RATE_HZ)
#define TICKS_PER_MS (FW_CONTROL_RATE_HZ / 1000u)
#define HEALTHY_MS 1000u
#define PGOOD_WINDOW 0.05f
#define PGOOD_MS 5u
#define STATUS_PERIOD_MS 100u
#define HEARTBEAT_MS 250u
#define MAX_CONSECUTIVE_WDT 3u
#define SETPOINT_SLEW_V_PER_MS 0.5f
#define CAN_BITRATE 500000u

static fw_params_t s_params;
static fw_status_t s_st;
static fw_ctrl_t s_ctrl;
static fw_prot_t s_prot;
static fw_ramp_t s_ramp;
static fw_retained_t *s_ret;

static volatile uint32_t s_isr_ticks;
static uint32_t s_last_isr_ticks;
static uint32_t s_healthy_ms;
static uint32_t s_pgood_ms;
static uint32_t s_last_telem_ms;
static uint32_t s_last_status_ms;
static uint32_t s_last_heartbeat_ms;
static volatile bool s_fault_event_pending;
static bool s_hang_requested;
static bool s_enable_prev;
static bool s_can_recovery_requested;

/* ------------------------------------------------------------- helpers */

static float code_to(uint16_t code, float full_scale)
{
    return (float)code * full_scale / (float)HAL_ADC_MAX_CODE;
}

static void sample(fw_meas_t *m)
{
    m->vout = code_to(hal_adc_read(HAL_ADC_VOUT), HAL_ADC_VOUT_FULL_SCALE);
    m->iout = code_to(hal_adc_read(HAL_ADC_IOUT), HAL_ADC_IOUT_FULL_SCALE);
    m->vin = code_to(hal_adc_read(HAL_ADC_VIN), HAL_ADC_VIN_FULL_SCALE);
    m->temp = code_to(hal_adc_read(HAL_ADC_TEMP), HAL_ADC_TEMP_FULL_SCALE);
}

static uint8_t lowest_bit(uint8_t bits)
{
    return (uint8_t)(bits & (uint8_t)(-(int)bits));
}

static void set_state(fw_state_t s)
{
    s_st.state = s;
    s_st.state_entered_ms = hal_millis();
}

static void retained_save(void)
{
    if (s_ret) {
        fw_retained_commit(s_ret);
    }
}

static void power_stage_off(void)
{
    hal_pwm_set_duty(0.0f);
    hal_pwm_enable(false);
    s_st.duty = 0.0f;
    s_st.i_ref = 0.0f;
    s_st.pgood = false;
    s_pgood_ms = 0;
    hal_gpio_write(HAL_PIN_PGOOD, false);
}

static bool switching(void)
{
    return s_st.state == FW_STATE_SOFT_START || s_st.state == FW_STATE_REGULATING;
}

static void enter_latched(uint8_t fault)
{
    power_stage_off();
    s_st.active_fault = fault;
    s_st.fault_history |= fault;
    hal_gpio_write(HAL_PIN_FAULT_LED, true);
    set_state(FW_STATE_LATCHED);
    if (s_ret) {
        s_ret->latched = 1;
        s_ret->last_fault = fault;
        s_ret->fault_history |= fault;
        retained_save();
    }
}

static void enter_fault(uint8_t faults)
{
    power_stage_off();
    uint8_t primary = lowest_bit(faults);
    s_st.active_fault = primary;
    s_st.fault_history |= faults;
    s_st.fault_count++;
    s_fault_event_pending = true;
    hal_gpio_write(HAL_PIN_FAULT_LED, true);
    if (s_ret) {
        s_ret->last_fault = primary;
        s_ret->fault_history |= faults;
        retained_save();
    }
    if (fw_prot_recovery(primary) == FW_RECOVERY_LATCH) {
        enter_latched(primary);
    } else {
        set_state(FW_STATE_FAULT);
    }
}

static void clear_fault_indication(void)
{
    s_st.active_fault = FW_FAULT_NONE;
    hal_gpio_write(HAL_PIN_FAULT_LED, false);
}

static void start_soft_start(void)
{
    uint8_t inhibit = fw_prot_start_inhibit(&s_prot, &s_st.meas);
    if (inhibit) {
        enter_fault(inhibit);
        return;
    }
    fw_ctrl_reset(&s_ctrl);
    fw_prot_reset_counters(&s_prot);

    /* Start the ramp from the present output (pre-biased start-up) so the
     * converter never actively discharges an already charged output. */
    float from = s_st.meas.vout;
    if (from < 0.0f) {
        from = 0.0f;
    }
    if (from > s_st.vref_target) {
        from = s_st.vref_target;
    }
    fw_ramp_start(&s_ramp, from, s_st.vref_target, s_params.soft_start_ms * TICKS_PER_MS);
    s_st.vref_now = s_ramp.value;
    clear_fault_indication();
    s_healthy_ms = 0;

    set_state(FW_STATE_SOFT_START);
    hal_pwm_set_duty(0.0f);
    hal_pwm_enable(true);
}

/* ------------------------------------------------------------ interrupts */

static void control_isr(void)
{
    s_isr_ticks++;
    sample(&s_st.meas);
    if (!switching()) {
        return;
    }
    uint8_t faults = fw_prot_fast(&s_prot, &s_st.meas);
    if (faults) {
        enter_fault(faults);
        return;
    }
    s_st.vref_now = fw_ramp_step(&s_ramp);
    float duty = fw_ctrl_step(&s_ctrl, s_st.vref_now, &s_st.meas);
    s_st.duty = duty;
    s_st.i_ref = s_ctrl.i_ref;
    hal_pwm_set_duty(duty);
}

static void gpio_isr(uint8_t pin, bool level)
{
    if (pin == HAL_PIN_ESTOP_N && !level) {
        /* The PWM break input already forced the switches off in hardware;
         * the ISR brings the software state in line. */
        power_stage_off();
        set_state(FW_STATE_ESTOP);
    }
}

/* -------------------------------------------------------------- commands */

int fw_cmd_set_vref(float volts)
{
    if (!(volts >= 1.0f && volts <= s_params.ovp_trip - 1.0f)) {
        return FW_ACK_OUT_OF_RANGE;
    }
    s_st.vref_target = volts;
    s_params.vref = volts;
    if (switching()) {
        float delta = volts - s_ramp.value;
        if (delta < 0.0f) {
            delta = -delta;
        }
        uint32_t steps = (uint32_t)(delta / SETPOINT_SLEW_V_PER_MS * (float)TICKS_PER_MS) + 1u;
        fw_ramp_start(&s_ramp, s_ramp.value, volts, steps);
        s_pgood_ms = 0;
    }
    return FW_ACK_OK;
}

int fw_cmd_set_current_limit(float amps)
{
    if (!(amps >= 0.5f && amps < s_params.ocp_trip)) {
        return FW_ACK_OUT_OF_RANGE;
    }
    s_params.current_limit = amps;
    s_st.current_limit = amps;
    fw_ctrl_set_current_limit(&s_ctrl, amps);
    return FW_ACK_OK;
}

int fw_cmd_clear_latch(void)
{
    if (s_st.state != FW_STATE_LATCHED && s_st.state != FW_STATE_FAULT) {
        return FW_ACK_BAD_STATE;
    }
    if (s_ret) {
        s_ret->latched = 0;
        s_ret->consecutive_wdt = 0;
        retained_save();
    }
    s_st.retries = 0;
    clear_fault_indication();
    set_state(FW_STATE_STANDBY);
    return FW_ACK_OK;
}

void fw_cmd_soft_reset(void)
{
    power_stage_off();
    hal_system_reset();
}

/* ---------------------------------------------------------------- CAN */

static void can_send_ack(uint8_t cmd, uint8_t status)
{
    hal_can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = (uint16_t)(FW_CAN_ID_ACK + s_params.can_node_id);
    f.dlc = 2;
    f.data[0] = cmd;
    f.data[1] = status;
    hal_can_send(&f);
}

static void can_handle_command(const hal_can_frame_t *f)
{
    s_st.can_commands++;
    if (f->dlc < 1) {
        can_send_ack(0, FW_ACK_BAD_LENGTH);
        return;
    }
    uint8_t cmd = f->data[0];
    uint16_t arg = f->dlc >= 3 ? fw_get_u16(&f->data[1]) : 0u;
    int status;
    switch (cmd) {
    case FW_CMD_SET_VREF:
        status = f->dlc >= 3 ? fw_cmd_set_vref((float)arg / 1000.0f) : FW_ACK_BAD_LENGTH;
        break;
    case FW_CMD_SET_CURRENT_LIMIT:
        status = f->dlc >= 3 ? fw_cmd_set_current_limit((float)arg / 1000.0f) : FW_ACK_BAD_LENGTH;
        break;
    case FW_CMD_CLEAR_LATCH:
        status = fw_cmd_clear_latch();
        break;
    case FW_CMD_SOFT_RESET:
        /* No ACK: the reset aborts pending frames; the BOOT frame confirms. */
        fw_cmd_soft_reset();
        return;
    case FW_CMD_HANG_TEST:
        s_hang_requested = true;
        status = FW_ACK_OK;
        break;
    default:
        status = FW_ACK_UNKNOWN;
        break;
    }
    can_send_ack(cmd, (uint8_t)status);
}

static void can_poll(void)
{
    if (hal_can_state() == HAL_CAN_BUS_OFF) {
        if (!s_can_recovery_requested) {
            hal_can_recover();
            s_can_recovery_requested = true;
            s_st.can_bus_off_recoveries++;
        }
        return;
    }
    s_can_recovery_requested = false;

    hal_can_frame_t f;
    while (hal_can_receive(&f)) {
        if (f.id == FW_CAN_ID_COMMAND + s_params.can_node_id) {
            can_handle_command(&f);
        }
    }
}

static void can_send_boot(void)
{
    hal_can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = (uint16_t)(FW_CAN_ID_BOOT + s_params.can_node_id);
    f.dlc = 4;
    f.data[0] = (uint8_t)s_st.reset_cause;
    fw_put_u16(&f.data[1], (uint16_t)s_st.boot_count);
    f.data[3] = s_st.params_from_flash ? 1u : 0u;
    hal_can_send(&f);
}

static void can_telemetry(uint32_t now)
{
    if (hal_can_state() == HAL_CAN_BUS_OFF) {
        return;
    }
    if (s_fault_event_pending) {
        hal_can_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id = (uint16_t)(FW_CAN_ID_FAULT + s_params.can_node_id);
        f.dlc = 6;
        f.data[0] = s_st.active_fault;
        f.data[1] = (uint8_t)s_st.state;
        fw_put_u16(&f.data[2], fw_to_u16(s_st.meas.vout, 1000.0f));
        fw_put_i16(&f.data[4], fw_to_i16(s_st.meas.iout, 1000.0f));
        if (hal_can_send(&f)) {
            s_fault_event_pending = false;
        }
    }
    if (now - s_last_telem_ms >= s_params.telemetry_period_ms) {
        s_last_telem_ms = now;
        fw_telem_t t;
        t.vout_mv = fw_to_u16(s_st.meas.vout, 1000.0f);
        t.iout_ma = fw_to_i16(s_st.meas.iout, 1000.0f);
        t.vin_dv = fw_to_u16(s_st.meas.vin, 10.0f);
        t.state = (uint8_t)s_st.state;
        t.fault = s_st.active_fault;
        hal_can_frame_t f;
        fw_encode_telem(&f, s_params.can_node_id, &t);
        if (hal_can_send(&f)) {
            s_st.telemetry_frames++;
        }
    }
    if (now - s_last_status_ms >= STATUS_PERIOD_MS) {
        s_last_status_ms = now;
        hal_can_frame_t f;
        memset(&f, 0, sizeof(f));
        f.id = (uint16_t)(FW_CAN_ID_STATUS + s_params.can_node_id);
        f.dlc = 8;
        fw_put_i16(&f.data[0], fw_to_i16(s_st.meas.temp, 10.0f));
        f.data[2] = s_st.retries;
        f.data[3] = (uint8_t)s_st.reset_cause;
        f.data[4] = (uint8_t)(s_st.wdt_resets > 255u ? 255u : s_st.wdt_resets);
        f.data[5] = s_st.fault_history;
        f.data[6] = (uint8_t)hal_can_state();
        f.data[7] = (uint8_t)((s_st.pgood ? 0x01u : 0u) | (s_st.params_from_flash ? 0x02u : 0u) |
                              (hal_gpio_read(HAL_PIN_ENABLE) ? 0x04u : 0u) |
                              (!hal_gpio_read(HAL_PIN_ESTOP_N) ? 0x08u : 0u));
        hal_can_send(&f);
    }
}

/* -------------------------------------------------------------- shell */

static bool parse_float(const char *s, float *out)
{
    char *end = NULL;
    float v = strtof(s, &end);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = v;
    return true;
}

static void print_faults(uint8_t bits)
{
    if (!bits) {
        fw_shell_printf(" NONE");
        return;
    }
    for (uint8_t b = 1; b != 0; b = (uint8_t)(b << 1)) {
        if (bits & b) {
            fw_shell_printf(" %s", fw_fault_name(b));
        }
    }
}

static void shell_handler(int argc, char **argv)
{
    const char *cmd = argv[0];
    if (strcmp(cmd, "help") == 0) {
        fw_shell_printf("OK commands: status ver params faults stats clear reset "
                        "set vref <V> | set ilim <A>\r\n");
    } else if (strcmp(cmd, "ver") == 0) {
        fw_shell_printf("OK %s\r\n", FW_VERSION_STRING);
    } else if (strcmp(cmd, "status") == 0) {
        fw_shell_printf("OK state=%s vout=%.3f iout=%.3f vin=%.2f temp=%.1f duty=%.3f vref=%.3f "
                        "pgood=%d fault=%s retries=%u\r\n",
                        fw_state_name(s_st.state), (double)s_st.meas.vout,
                        (double)s_st.meas.iout, (double)s_st.meas.vin, (double)s_st.meas.temp,
                        (double)s_st.duty, (double)s_st.vref_target, s_st.pgood ? 1 : 0,
                        fw_fault_name(s_st.active_fault), (unsigned)s_st.retries);
    } else if (strcmp(cmd, "params") == 0) {
        fw_shell_printf("OK source=%s vref=%.3f kp_v=%.3f ki_v=%.1f kp_i=%.3f ilim=%.2f "
                        "ocp=%.2f ovp=%.2f retries=%u wdt_ms=%u\r\n",
                        s_st.params_from_flash ? "flash" : "defaults", (double)s_params.vref,
                        (double)s_params.kp_v, (double)s_params.ki_v, (double)s_params.kp_i,
                        (double)s_params.current_limit, (double)s_params.ocp_trip,
                        (double)s_params.ovp_trip, (unsigned)s_params.max_retries,
                        (unsigned)s_params.wdt_timeout_ms);
    } else if (strcmp(cmd, "faults") == 0) {
        fw_shell_printf("OK active=%s history=", fw_fault_name(s_st.active_fault));
        print_faults(s_st.fault_history);
        fw_shell_printf(" count=%u\r\n", (unsigned)s_st.fault_count);
    } else if (strcmp(cmd, "stats") == 0) {
        const fw_shell_stats_t *ss = fw_shell_stats();
        fw_shell_printf("OK boots=%u wdt_resets=%u reset_cause=%u can_cmds=%u telem=%u "
                        "uart_errors=%u discarded=%u\r\n",
                        (unsigned)s_st.boot_count, (unsigned)s_st.wdt_resets,
                        (unsigned)s_st.reset_cause, (unsigned)s_st.can_commands,
                        (unsigned)s_st.telemetry_frames, (unsigned)ss->uart_errors,
                        (unsigned)ss->discarded);
    } else if (strcmp(cmd, "clear") == 0) {
        int rc = fw_cmd_clear_latch();
        fw_shell_printf(rc == FW_ACK_OK ? "OK cleared\r\n" : "ERR nothing to clear\r\n");
    } else if (strcmp(cmd, "reset") == 0) {
        fw_shell_printf("OK resetting\r\n");
        fw_shell_flush();
        fw_cmd_soft_reset();
    } else if (strcmp(cmd, "set") == 0 && argc == 3) {
        float v;
        if (!parse_float(argv[2], &v)) {
            fw_shell_printf("ERR bad number\r\n");
            return;
        }
        int rc;
        if (strcmp(argv[1], "vref") == 0) {
            rc = fw_cmd_set_vref(v);
        } else if (strcmp(argv[1], "ilim") == 0) {
            rc = fw_cmd_set_current_limit(v);
        } else {
            fw_shell_printf("ERR unknown parameter\r\n");
            return;
        }
        if (rc == FW_ACK_OK) {
            fw_shell_printf("OK %s=%.3f\r\n", argv[1], (double)v);
        } else {
            fw_shell_printf("ERR out of range\r\n");
        }
    } else {
        fw_shell_printf("ERR unknown command '%s'\r\n", cmd);
    }
}

/* ------------------------------------------------------------- boot */

static void load_params(void)
{
    fw_params_t p;
    size_t n = hal_flash_read(FW_PARAMS_FLASH_OFFSET, &p, sizeof(p));
    if (n == sizeof(p) && fw_params_valid(&p) && fw_params_sane(&p)) {
        s_params = p;
        s_st.params_from_flash = true;
    } else {
        fw_params_defaults(&s_params);
        s_st.params_from_flash = false;
    }
}

static void boot_retained(void)
{
    bool valid = false;
    s_ret = fw_retained_attach(&valid);
    if (!s_ret) {
        return;
    }
    if (!valid || s_st.reset_cause == HAL_RESET_POWER_ON) {
        fw_retained_format(s_ret);
    }
    s_ret->boot_count++;
    switch (s_st.reset_cause) {
    case HAL_RESET_WATCHDOG:
        s_ret->wdt_resets++;
        s_ret->consecutive_wdt++;
        s_ret->fault_history |= FW_FAULT_WATCHDOG;
        break;
    case HAL_RESET_SOFTWARE:
        s_ret->sw_resets++;
        break;
    case HAL_RESET_BROWNOUT:
        s_ret->brownouts++;
        break;
    case HAL_RESET_POWER_ON:
        break;
    }
    retained_save();
    s_st.boot_count = s_ret->boot_count;
    s_st.wdt_resets = s_ret->wdt_resets;
    s_st.fault_history = s_ret->fault_history;
}

void fw_init(void)
{
    memset(&s_st, 0, sizeof(s_st));
    s_isr_ticks = 0;
    s_last_isr_ticks = 0;
    s_healthy_ms = 0;
    s_pgood_ms = 0;
    s_last_telem_ms = 0;
    s_last_status_ms = 0;
    s_last_heartbeat_ms = 0;
    s_fault_event_pending = false;
    s_hang_requested = false;
    s_can_recovery_requested = false;
    s_ret = NULL;

    s_st.reset_cause = hal_reset_cause();
    load_params();
    s_st.vref_target = s_params.vref;
    s_st.current_limit = s_params.current_limit;
    fw_ctrl_init(&s_ctrl, s_params.kp_v, s_params.ki_v, s_params.kp_i, s_params.current_limit,
                 CONTROL_TS);
    fw_prot_init(&s_prot, &s_params);

    /* GPIO */
    hal_gpio_config_output(HAL_PIN_FAULT_LED, false);
    hal_gpio_config_output(HAL_PIN_PGOOD, false);
    hal_gpio_config_output(HAL_PIN_HEARTBEAT, false);
    hal_gpio_config_input(HAL_PIN_ENABLE, false, true);
    hal_gpio_config_input(HAL_PIN_ESTOP_N, true, false);
    hal_gpio_set_isr(gpio_isr);
    hal_gpio_enable_irq(HAL_PIN_ESTOP_N, HAL_EDGE_FALLING);

    /* Power stage starts off. */
    hal_pwm_set_duty(0.0f);
    hal_pwm_enable(false);

    /* Communication */
    hal_uart_init();
    fw_shell_init(shell_handler);
    hal_can_init(CAN_BITRATE);
    hal_can_set_filter(0, (uint16_t)(FW_CAN_ID_COMMAND + s_params.can_node_id), 0x7FFu);

    boot_retained();
    sample(&s_st.meas);
    s_enable_prev = hal_gpio_read(HAL_PIN_ENABLE);

    set_state(FW_STATE_STANDBY);
    if (s_ret && s_ret->latched) {
        enter_latched(s_ret->last_fault);
    } else if (s_ret && s_ret->consecutive_wdt >= MAX_CONSECUTIVE_WDT) {
        /* Reset loop: stop auto-restarting and wait for an operator. */
        enter_latched(FW_FAULT_WATCHDOG);
    } else if (!hal_gpio_read(HAL_PIN_ESTOP_N)) {
        set_state(FW_STATE_ESTOP);
    }

    hal_set_control_isr(control_isr, FW_CONTROL_RATE_HZ);
    can_send_boot();
    fw_shell_printf("\r\n%s boot=%u reset=%u params=%s\r\n", FW_VERSION_STRING,
                    (unsigned)s_st.boot_count, (unsigned)s_st.reset_cause,
                    s_st.params_from_flash ? "flash" : "defaults");
    fw_shell_flush();

    hal_wdt_start(s_params.wdt_timeout_ms);
}

/* --------------------------------------------------------- main loop */

static void run_state_machine(uint32_t now, bool enable, bool estop)
{
    bool enable_fell = s_enable_prev && !enable;
    s_enable_prev = enable;

    if (estop && s_st.state != FW_STATE_ESTOP) {
        power_stage_off();
        set_state(FW_STATE_ESTOP);
        return;
    }

    switch (s_st.state) {
    case FW_STATE_STANDBY:
        if (enable) {
            start_soft_start();
        }
        break;

    case FW_STATE_SOFT_START:
    case FW_STATE_REGULATING: {
        uint8_t faults = fw_prot_slow(&s_prot, &s_st.meas, s_st.duty, s_st.vref_now,
                                      s_st.state == FW_STATE_REGULATING);
        if (faults) {
            enter_fault(faults);
        } else if (!enable) {
            power_stage_off();
            set_state(FW_STATE_STANDBY);
        } else if (s_st.state == FW_STATE_SOFT_START && fw_ramp_done(&s_ramp)) {
            set_state(FW_STATE_REGULATING);
            s_healthy_ms = 0;
        } else if (s_st.state == FW_STATE_REGULATING) {
            if (s_healthy_ms < HEALTHY_MS) {
                s_healthy_ms++;
                if (s_healthy_ms == HEALTHY_MS) {
                    /* A full second of clean regulation earns the retry
                     * budget back and ends any watchdog reset streak. */
                    s_st.retries = 0;
                    if (s_ret && s_ret->consecutive_wdt) {
                        s_ret->consecutive_wdt = 0;
                        retained_save();
                    }
                }
            }
        }
        break;
    }

    case FW_STATE_FAULT: {
        if (!enable) {
            s_st.retries = 0;
            clear_fault_indication();
            set_state(FW_STATE_STANDBY);
            break;
        }
        if (now - s_st.state_entered_ms < s_params.retry_delay_ms) {
            break;
        }
        uint8_t fault = s_st.active_fault;
        fw_recovery_t rec = fw_prot_recovery(fault);
        if (rec == FW_RECOVERY_WAIT_CLEAR) {
            if (fw_prot_condition_cleared(&s_prot, fault, &s_st.meas)) {
                start_soft_start();
            }
        } else if (rec == FW_RECOVERY_RETRY) {
            if (s_st.retries >= s_params.max_retries) {
                enter_latched(fault);
            } else {
                s_st.retries++;
                start_soft_start();
            }
        } else {
            enter_latched(fault);
        }
        break;
    }

    case FW_STATE_LATCHED:
        if (enable_fell) {
            fw_cmd_clear_latch();
        }
        break;

    case FW_STATE_ESTOP:
        if (!estop && !enable) {
            /* Restart interlock: enable must be low before leaving e-stop. */
            set_state(FW_STATE_STANDBY);
        }
        break;
    }
}

static void update_pgood(void)
{
    bool in_window = false;
    if (s_st.state == FW_STATE_REGULATING && s_st.vref_target > 0.0f) {
        float err = s_st.meas.vout - s_st.vref_target;
        if (err < 0.0f) {
            err = -err;
        }
        in_window = err < PGOOD_WINDOW * s_st.vref_target;
    }
    if (in_window) {
        if (s_pgood_ms < PGOOD_MS) {
            s_pgood_ms++;
        }
    } else {
        s_pgood_ms = 0;
    }
    bool pgood = s_pgood_ms >= PGOOD_MS;
    if (pgood != s_st.pgood) {
        s_st.pgood = pgood;
        hal_gpio_write(HAL_PIN_PGOOD, pgood);
    }
}

void fw_main_tick(void)
{
    if (s_hang_requested) {
        /* Simulates the super-loop stuck in a busy-wait: nothing below runs,
         * in particular the watchdog is no longer serviced. */
        return;
    }
    uint32_t now = hal_millis();

    /* Only service the watchdog if the control interrupt is alive too. */
    if (s_isr_ticks != s_last_isr_ticks) {
        s_last_isr_ticks = s_isr_ticks;
        hal_wdt_kick();
    }

    bool enable = hal_gpio_read(HAL_PIN_ENABLE);
    bool estop = !hal_gpio_read(HAL_PIN_ESTOP_N);
    run_state_machine(now, enable, estop);
    update_pgood();

    if (now - s_last_heartbeat_ms >= HEARTBEAT_MS) {
        s_last_heartbeat_ms = now;
        hal_gpio_toggle(HAL_PIN_HEARTBEAT);
    }

    can_poll();
    can_telemetry(now);
    fw_shell_poll();
}

/* --------------------------------------------------------- accessors */

const fw_status_t *fw_status(void)
{
    return &s_st;
}

const fw_params_t *fw_active_params(void)
{
    return &s_params;
}

const char *fw_state_name(fw_state_t s)
{
    switch (s) {
    case FW_STATE_STANDBY: return "STANDBY";
    case FW_STATE_SOFT_START: return "SOFT_START";
    case FW_STATE_REGULATING: return "REGULATING";
    case FW_STATE_FAULT: return "FAULT";
    case FW_STATE_LATCHED: return "LATCHED";
    case FW_STATE_ESTOP: return "ESTOP";
    }
    return "UNKNOWN";
}
