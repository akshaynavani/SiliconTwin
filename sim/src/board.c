#include "sim/board.h"

#include <string.h>

void board_default_config(board_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    plant_default_params(&cfg->plant);
    cfg->seed = 1;
    cfg->step_us = 10;
    cfg->plant_substeps = 10;
    cfg->uart_baud = 115200;
    cfg->can_bitrate = 500000;
    cfg->pwm_freq_hz = 100000;
    cfg->adc_noise_lsb = 1.0;
}

void board_init(sim_board_t *b, const board_config_t *cfg, const mcu_firmware_t *fw)
{
    memset(b, 0, sizeof(*b));
    b->cfg = *cfg;
    if (b->cfg.step_us == 0) {
        b->cfg.step_us = 10;
    }
    if (b->cfg.plant_substeps == 0) {
        b->cfg.plant_substeps = 1;
    }
    sim_clock_init(&b->clock);
    sim_rng_seed(&b->rng, cfg->seed);
    plant_init(&b->plant, &cfg->plant);
    can_bus_init(&b->can, cfg->can_bitrate, &b->rng);
    b->mcu_node = (uint8_t)can_bus_attach(&b->can, "mcu");
    b->bench_node = (uint8_t)can_bus_attach(&b->can, "bench");
    mcu_init(&b->mcu, &b->clock, &b->rng, cfg->uart_baud, cfg->pwm_freq_hz, &b->can, b->mcu_node, fw);
    adc_set_noise(&b->mcu.adc, cfg->adc_noise_lsb);

    /* Bench defaults: e-stop released (high), converter disabled. */
    gpio_drive_external(&b->mcu.gpio, HAL_PIN_ESTOP_N, true);
    gpio_drive_external(&b->mcu.gpio, HAL_PIN_ENABLE, false);
}

static void sample_analog(sim_board_t *b)
{
    adc_t *adc = &b->mcu.adc;
    adc_set_input(adc, HAL_ADC_VOUT, b->plant.v_out);
    adc_set_input(adc, HAL_ADC_IOUT, b->plant.i_l);
    adc_set_input(adc, HAL_ADC_VIN, b->plant.p.vin);
    adc_set_input(adc, HAL_ADC_TEMP, b->plant.temp);
}

void board_power_on(sim_board_t *b)
{
    sample_analog(b);
    mcu_power_on(&b->mcu);
}

void board_step(sim_board_t *b)
{
    double duty = pwm_effective_duty(&b->mcu.pwm);
    double dt = (double)b->cfg.step_us * 1e-6 / (double)b->cfg.plant_substeps;
    for (uint32_t i = 0; i < b->cfg.plant_substeps; i++) {
        plant_step(&b->plant, duty, dt);
    }

    sim_clock_advance(&b->clock, b->cfg.step_us);
    sample_analog(b);

    /* The e-stop line is hard-wired to the PWM timer's break input. */
    pwm_set_break(&b->mcu.pwm, !gpio_read(&b->mcu.gpio, HAL_PIN_ESTOP_N));

    uart_step(&b->mcu.uart, b->clock.now);
    can_bus_step(&b->can, b->clock.now);
    mcu_step(&b->mcu);
}

void board_run_for(sim_board_t *b, sim_time_us duration)
{
    sim_time_us end = b->clock.now + duration;
    while (b->clock.now < end) {
        board_step(b);
    }
}

sim_time_us board_now(const sim_board_t *b)
{
    return b->clock.now;
}

void board_set_enable(sim_board_t *b, bool on)
{
    gpio_drive_external(&b->mcu.gpio, HAL_PIN_ENABLE, on);
}

void board_set_estop(sim_board_t *b, bool asserted)
{
    /* Active low. */
    gpio_drive_external(&b->mcu.gpio, HAL_PIN_ESTOP_N, !asserted);
}

bool board_bench_can_send(sim_board_t *b, uint16_t id, const uint8_t *data, uint8_t dlc)
{
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id = id;
    f.dlc = dlc > 8 ? 8 : dlc;
    if (data && f.dlc) {
        memcpy(f.data, data, f.dlc);
    }
    return can_node_send(&b->can, b->bench_node, &f);
}

bool board_bench_can_receive(sim_board_t *b, can_frame_t *frame)
{
    return can_node_receive(&b->can, b->bench_node, frame);
}

size_t board_uart_send(sim_board_t *b, const char *text)
{
    return uart_host_send(&b->mcu.uart, text, strlen(text));
}

void board_inject_fault(sim_board_t *b, const sim_fault_t *fault)
{
    if ((int)fault->kind < 0 || fault->kind >= SIM_FAULT_COUNT) {
        return;
    }
    b->fault_injections++;
    b->active_faults[fault->kind] = true;

    switch (fault->kind) {
    case SIM_FAULT_OUTPUT_SHORT:
        plant_set_short(&b->plant, true, fault->a > 0.0 ? fault->a : 0.02);
        break;
    case SIM_FAULT_ADC_STUCK:
        adc_inject_stuck(&b->mcu.adc, (uint8_t)fault->a, (uint16_t)fault->b);
        break;
    case SIM_FAULT_ADC_OFFSET:
        adc_inject_offset(&b->mcu.adc, (uint8_t)fault->a, fault->b);
        break;
    case SIM_FAULT_CAN_ERRORS:
        can_bus_set_error_rate(&b->can, fault->a);
        break;
    case SIM_FAULT_CAN_BUS_OFF:
        can_node_force_bus_off(&b->can, b->mcu_node);
        b->active_faults[fault->kind] = false; /* one-shot */
        break;
    case SIM_FAULT_UART_NOISE:
        uart_set_noise(&b->mcu.uart, fault->a);
        break;
    case SIM_FAULT_FW_HANG:
        mcu_hang(&b->mcu, SIM_MS(fault->a > 0.0 ? fault->a : 0.0));
        b->active_faults[fault->kind] = false; /* cleared by reset or timeout */
        break;
    case SIM_FAULT_GPIO_STUCK:
        gpio_inject_stuck(&b->mcu.gpio, (uint8_t)fault->a, fault->b != 0.0);
        break;
    case SIM_FAULT_BROWNOUT:
        mcu_reset(&b->mcu, HAL_RESET_BROWNOUT);
        b->active_faults[fault->kind] = false; /* one-shot */
        break;
    case SIM_FAULT_OVERTEMP:
        plant_set_ambient(&b->plant, fault->a);
        break;
    case SIM_FAULT_COUNT:
        break;
    }
}

void board_clear_fault(sim_board_t *b, const sim_fault_t *fault)
{
    if ((int)fault->kind < 0 || fault->kind >= SIM_FAULT_COUNT) {
        return;
    }
    b->active_faults[fault->kind] = false;

    switch (fault->kind) {
    case SIM_FAULT_OUTPUT_SHORT:
        plant_set_short(&b->plant, false, 0.0);
        break;
    case SIM_FAULT_ADC_STUCK:
    case SIM_FAULT_ADC_OFFSET:
        adc_clear_faults(&b->mcu.adc, (uint8_t)fault->a);
        break;
    case SIM_FAULT_CAN_ERRORS:
        can_bus_set_error_rate(&b->can, 0.0);
        break;
    case SIM_FAULT_UART_NOISE:
        uart_set_noise(&b->mcu.uart, 0.0);
        break;
    case SIM_FAULT_FW_HANG:
        mcu_unhang(&b->mcu);
        break;
    case SIM_FAULT_GPIO_STUCK:
        gpio_clear_stuck(&b->mcu.gpio, (uint8_t)fault->a);
        break;
    case SIM_FAULT_OVERTEMP:
        plant_set_ambient(&b->plant, b->cfg.plant.t_ambient);
        break;
    case SIM_FAULT_CAN_BUS_OFF:
    case SIM_FAULT_BROWNOUT:
    case SIM_FAULT_COUNT:
        break;
    }
}
