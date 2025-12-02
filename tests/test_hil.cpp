// Firmware-in-the-loop tests: the production firmware runs unmodified on the
// simulated MCU against the converter plant, CAN bus and UART bench.
#include "sim_fixture.h"

using Hil = BoardFixture;

// ------------------------------------------------------------- regulation
TEST_F(Hil, StaysOffWhileDisabled) {
    boot(/*enable=*/false);
    run_ms(50);
    EXPECT_EQ(st()->state, FW_STATE_STANDBY);
    EXPECT_FALSE(b()->mcu.pwm.enabled);
    EXPECT_LT(vout(), 0.1);
}

TEST_F(Hil, SoftStartsIntoRegulationWithoutOvershoot) {
    boot();
    double peak = 0;
    double t = run_until([&] {
        if (vout() > peak) peak = vout();
        return st()->pgood;
    }, 100);
    EXPECT_GE(t, static_cast<double>(params_.soft_start_ms));
    EXPECT_LT(t, 40.0);
    run_ms(30);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
    EXPECT_NEAR(vout(), 12.0, 0.1);
    EXPECT_LT(peak, 12.0 * 1.03);
    EXPECT_TRUE(gpio_read(&b()->mcu.gpio, HAL_PIN_PGOOD));
    EXPECT_FALSE(gpio_read(&b()->mcu.gpio, HAL_PIN_FAULT_LED));
}

TEST_F(Hil, RejectsFiftyPercentLoadStep) {
    boot();
    run_ms(60);
    plant_set_load(&b()->plant, 1.6);  // 5 A -> 7.5 A
    double vmin = 99;
    for (int i = 0; i < 300; i++) {
        run_ms(0.1);
        if (vout() < vmin) vmin = vout();
    }
    EXPECT_GT(vmin, 12.0 * 0.92);
    EXPECT_NEAR(vout(), 12.0, 0.12);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
}

TEST_F(Hil, DisableShutsDownCleanly) {
    boot();
    run_ms(60);
    board_set_enable(b(), false);
    run_ms(5);
    EXPECT_EQ(st()->state, FW_STATE_STANDBY);
    EXPECT_FALSE(b()->mcu.pwm.enabled);
    EXPECT_FALSE(st()->pgood);
}

TEST_F(Hil, HeartbeatToggles) {
    boot();
    uint32_t before = b()->mcu.gpio.pins[HAL_PIN_HEARTBEAT].toggle_count;
    run_ms(1000);
    uint32_t toggles = b()->mcu.gpio.pins[HAL_PIN_HEARTBEAT].toggle_count - before;
    EXPECT_GE(toggles, 3u);
    EXPECT_LE(toggles, 5u);
}

// ------------------------------------------------------------- protection
TEST_F(Hil, ShortCircuitRetriesThenLatchesOff) {
    boot();
    run_ms(50);
    sim_fault_t shorted{SIM_FAULT_OUTPUT_SHORT, 0.02, 0};
    board_inject_fault(b(), &shorted);
    run_ms(800);
    EXPECT_EQ(st()->state, FW_STATE_LATCHED);
    EXPECT_TRUE(st()->fault_history & FW_FAULT_OCP);
    EXPECT_EQ(st()->retries, params_.max_retries);
    EXPECT_TRUE(gpio_read(&b()->mcu.gpio, HAL_PIN_FAULT_LED));
    EXPECT_FALSE(frames_with_base(FW_CAN_ID_FAULT).empty());

    // Remains latched even with the short gone, until the operator cycles enable.
    board_clear_fault(b(), &shorted);
    run_ms(300);
    EXPECT_EQ(st()->state, FW_STATE_LATCHED);
    board_set_enable(b(), false);
    run_ms(5);
    board_set_enable(b(), true);
    run_ms(60);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
    EXPECT_NEAR(vout(), 12.0, 0.15);
}

TEST_F(Hil, CurrentLimitHoldsInductorCurrentOnOverload) {
    boot();
    run_ms(50);
    plant_set_load(&b()->plant, 0.8);  // would draw 15 A at 12 V
    run_ms(10);
    EXPECT_LT(b()->plant.i_l, params_.current_limit + 0.5);
    EXPECT_LT(vout(), 12.0 * 0.8);
}

TEST_F(Hil, InputSagWaitsForRecoveryWithoutConsumingRetries) {
    boot();
    run_ms(50);
    plant_set_vin(&b()->plant, 15.0);
    run_ms(300);
    EXPECT_EQ(st()->state, FW_STATE_FAULT);
    EXPECT_EQ(st()->active_fault, FW_FAULT_VIN_UV);
    EXPECT_EQ(st()->retries, 0);
    plant_set_vin(&b()->plant, 48.0);
    run_ms(60);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
}

TEST_F(Hil, StuckVoltageSensorLatchesWithoutRestart) {
    boot();
    run_ms(60);
    sim_fault_t stuck{SIM_FAULT_ADC_STUCK, HAL_ADC_VOUT, 0};
    board_inject_fault(b(), &stuck);
    run_ms(20);
    EXPECT_EQ(st()->state, FW_STATE_LATCHED);
    EXPECT_EQ(st()->active_fault, FW_FAULT_SENSOR);
    run_ms(500);
    EXPECT_EQ(st()->state, FW_STATE_LATCHED);
    EXPECT_FALSE(b()->mcu.pwm.enabled);
}

TEST_F(Hil, EmergencyStopActsWithinOneStepAndNeedsEnableCycle) {
    boot();
    run_ms(60);
    board_set_estop(b(), true);
    EXPECT_EQ(pwm_effective_duty(&b()->mcu.pwm), 0.0);  // ISR ran synchronously
    EXPECT_EQ(st()->state, FW_STATE_ESTOP);
    board_set_estop(b(), false);
    run_ms(50);
    EXPECT_EQ(st()->state, FW_STATE_ESTOP);  // restart interlock
    board_set_enable(b(), false);
    run_ms(2);
    EXPECT_EQ(st()->state, FW_STATE_STANDBY);
    board_set_enable(b(), true);
    run_ms(60);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
}

TEST_F(Hil, OverTemperatureRestartsAfterCooling) {
    cfg_.plant.tau_thermal = 0.05;  // speed up the thermal transient
    boot();
    run_ms(50);
    sim_fault_t hot{SIM_FAULT_OVERTEMP, 150.0, 0};
    board_inject_fault(b(), &hot);
    run_ms(300);
    EXPECT_EQ(st()->active_fault, FW_FAULT_OTP);
    board_clear_fault(b(), &hot);
    run_until([&] { return st()->state == FW_STATE_REGULATING; }, 1000);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
    EXPECT_LT(b()->plant.temp, params_.otp_clear);
}

// ---------------------------------------------------- watchdog & resets
TEST_F(Hil, WatchdogResetsHungFirmwareWhichRecovers) {
    boot();
    run_ms(60);
    mcu_hang(&b()->mcu, 0);
    run_ms(params_.wdt_timeout_ms + 5);
    EXPECT_EQ(b()->mcu.reset_count, 2u);  // power-on + watchdog
    EXPECT_EQ(b()->mcu.reset_cause, HAL_RESET_WATCHDOG);
    EXPECT_EQ(st()->reset_cause, HAL_RESET_WATCHDOG);
    EXPECT_EQ(st()->wdt_resets, 1u);
    EXPECT_EQ(st()->boot_count, 2u);
    run_ms(60);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
    EXPECT_EQ(frames_with_base(FW_CAN_ID_BOOT).size(), 1u);  // frames_ cleared at boot
}

TEST_F(Hil, WatchdogIsServicedOnlyWhileControlIsrRuns) {
    boot();
    run_ms(30);
    hal_sim_bind(&b()->mcu);
    hal_set_control_isr(nullptr, 0);  // control interrupt dies, main loop keeps going
    run_ms(params_.wdt_timeout_ms + 5);
    EXPECT_EQ(b()->mcu.wdt.expirations, 1u);
    EXPECT_EQ(st()->reset_cause, HAL_RESET_WATCHDOG);
}

TEST_F(Hil, ConsecutiveWatchdogResetsLatchOff) {
    boot();
    for (int i = 0; i < 3; i++) {
        run_ms(100);
        mcu_hang(&b()->mcu, 0);
    }
    run_ms(100);
    EXPECT_EQ(b()->mcu.wdt.expirations, 3u);
    EXPECT_EQ(st()->state, FW_STATE_LATCHED);
    EXPECT_EQ(st()->active_fault, FW_FAULT_WATCHDOG);
}

TEST_F(Hil, HealthyIntervalEndsWatchdogStreak) {
    boot();
    run_ms(100);
    mcu_hang(&b()->mcu, 0);
    run_ms(1300);  // reboot + more than a second of clean regulation
    mcu_hang(&b()->mcu, 0);
    run_ms(150);
    mcu_hang(&b()->mcu, 0);
    run_ms(150);
    EXPECT_EQ(b()->mcu.wdt.expirations, 3u);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
}

TEST_F(Hil, LatchSurvivesWatchdogReset) {
    boot();
    run_ms(60);
    sim_fault_t stuck{SIM_FAULT_ADC_STUCK, HAL_ADC_VOUT, 0};
    board_inject_fault(b(), &stuck);
    run_ms(20);
    ASSERT_EQ(st()->state, FW_STATE_LATCHED);
    mcu_hang(&b()->mcu, 0);
    run_ms(100);
    EXPECT_EQ(st()->reset_cause, HAL_RESET_WATCHDOG);
    EXPECT_EQ(st()->state, FW_STATE_LATCHED);
    EXPECT_EQ(st()->active_fault, FW_FAULT_SENSOR);
}

TEST_F(Hil, BrownoutAndSoftwareResetsAreDistinguished) {
    boot();
    run_ms(60);
    sim_fault_t bo{SIM_FAULT_BROWNOUT, 0, 0};
    board_inject_fault(b(), &bo);
    EXPECT_EQ(st()->reset_cause, HAL_RESET_BROWNOUT);
    run_ms(60);
    can_cmd(FW_CMD_SOFT_RESET);
    run_ms(5);
    EXPECT_EQ(st()->reset_cause, HAL_RESET_SOFTWARE);
    EXPECT_EQ(st()->boot_count, 3u);
}

TEST_F(Hil, PowerOnClearsRetainedState) {
    boot();
    run_ms(10);
    EXPECT_EQ(st()->boot_count, 1u);
    EXPECT_EQ(st()->wdt_resets, 0u);
}

// ------------------------------------------------------------- parameters
TEST_F(Hil, UsesCalibrationFromFlash) {
    params_.vref = 5.0f;
    params_.ovp_trip = 6.5f;
    boot();
    run_ms(80);
    EXPECT_TRUE(st()->params_from_flash);
    EXPECT_NEAR(vout(), 5.0, 0.1);
}

TEST_F(Hil, CorruptFlashFallsBackToDefaults) {
    params_.vref = 5.0f;
    boot(true, true, /*corrupt=*/true);
    run_ms(80);
    EXPECT_FALSE(st()->params_from_flash);
    EXPECT_NEAR(vout(), 12.0, 0.15);
}

TEST_F(Hil, BlankFlashFallsBackToDefaults) {
    boot(true, /*flash=*/false);
    run_ms(10);
    EXPECT_FALSE(st()->params_from_flash);
}

TEST_F(Hil, InsaneButValidImageIsRejected) {
    params_.vref = 12.0f;
    params_.ovp_trip = 10.0f;  // correct CRC, unsafe content
    boot();
    run_ms(10);
    EXPECT_FALSE(st()->params_from_flash);
    EXPECT_FLOAT_EQ(fw_active_params()->ovp_trip, 14.5f);
}

// -------------------------------------------------------------------- CAN
TEST_F(Hil, BootFramePublishesResetCause) {
    boot();
    run_ms(5);
    auto boots = frames_with_base(FW_CAN_ID_BOOT);
    ASSERT_EQ(boots.size(), 1u);
    EXPECT_EQ(boots[0].data[0], HAL_RESET_POWER_ON);
    EXPECT_EQ(boots[0].data[3], 1u);  // params from flash
}

TEST_F(Hil, TelemetryMatchesPlantState) {
    boot();
    run_ms(100);
    auto telem = frames_with_base(FW_CAN_ID_TELEM);
    EXPECT_GE(telem.size(), 9u);
    const can_frame_t &last = telem.back();
    double v = fw_get_u16(&last.data[0]) / 1000.0;
    EXPECT_NEAR(v, vout(), 0.1);
    EXPECT_EQ(last.data[6], FW_STATE_REGULATING);
    EXPECT_FALSE(frames_with_base(FW_CAN_ID_STATUS).empty());
}

TEST_F(Hil, SetpointCommandIsAcknowledgedAndApplied) {
    boot();
    run_ms(50);
    can_cmd(FW_CMD_SET_VREF, 9000, true);
    run_ms(40);
    auto acks = frames_with_base(FW_CAN_ID_ACK);
    ASSERT_EQ(acks.size(), 1u);
    EXPECT_EQ(acks[0].data[0], FW_CMD_SET_VREF);
    EXPECT_EQ(acks[0].data[1], FW_ACK_OK);
    EXPECT_NEAR(vout(), 9.0, 0.1);
}

TEST_F(Hil, InvalidCommandsAreRejected) {
    boot();
    run_ms(50);
    can_cmd(FW_CMD_SET_VREF, 30000, true);  // above OVP margin
    run_ms(2);
    can_cmd(0x55);
    run_ms(2);
    can_cmd(FW_CMD_SET_VREF);  // missing argument
    run_ms(2);
    can_cmd(FW_CMD_CLEAR_LATCH);  // nothing latched
    run_ms(5);
    auto acks = frames_with_base(FW_CAN_ID_ACK);
    ASSERT_EQ(acks.size(), 4u);
    EXPECT_EQ(acks[0].data[1], FW_ACK_OUT_OF_RANGE);
    EXPECT_EQ(acks[1].data[1], FW_ACK_UNKNOWN);
    EXPECT_EQ(acks[2].data[1], FW_ACK_BAD_LENGTH);
    EXPECT_EQ(acks[3].data[1], FW_ACK_BAD_STATE);
    EXPECT_NEAR(st()->vref_target, 12.0f, 1e-3);
}

TEST_F(Hil, ClearLatchOverCan) {
    boot();
    run_ms(50);
    sim_fault_t stuck{SIM_FAULT_ADC_STUCK, HAL_ADC_VOUT, 0};
    board_inject_fault(b(), &stuck);
    run_ms(20);
    ASSERT_EQ(st()->state, FW_STATE_LATCHED);
    board_clear_fault(b(), &stuck);
    can_cmd(FW_CMD_CLEAR_LATCH);
    run_ms(60);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);  // enable still high -> restarts
}

TEST_F(Hil, HangCommandTriggersWatchdog) {
    boot();
    run_ms(50);
    can_cmd(FW_CMD_HANG_TEST);
    run_ms(params_.wdt_timeout_ms + 10);
    EXPECT_EQ(st()->reset_cause, HAL_RESET_WATCHDOG);
}

TEST_F(Hil, RecoversFromBusOffAndResumesTelemetry) {
    boot();
    run_ms(50);
    can_node_force_bus_off(&b()->can, b()->mcu_node);
    run_ms(10);
    EXPECT_EQ(st()->can_bus_off_recoveries, 1u);
    EXPECT_EQ(can_node_state(&b()->can, b()->mcu_node), CAN_STATE_ERROR_ACTIVE);
    frames_.clear();
    run_ms(50);
    EXPECT_GE(frames_with_base(FW_CAN_ID_TELEM).size(), 4u);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);  // power path unaffected
}

TEST_F(Hil, NoisyBusDoesNotDisturbRegulation) {
    boot();
    sim_fault_t noisy{SIM_FAULT_CAN_ERRORS, 0.2, 0};
    board_inject_fault(b(), &noisy);
    run_ms(300);
    EXPECT_GT(b()->can.error_frames, 0u);
    EXPECT_EQ(st()->state, FW_STATE_REGULATING);
    EXPECT_FALSE(frames_with_base(FW_CAN_ID_TELEM).empty());
}

// ------------------------------------------------------------------- UART
TEST_F(Hil, ShellPrintsBannerAndStatus) {
    boot();
    run_ms(60);
    uart("status\n");
    run_ms(30);
    std::string out = uart_text();
    EXPECT_NE(out.find(FW_VERSION_STRING), std::string::npos);
    EXPECT_NE(out.find("OK state=REGULATING"), std::string::npos);
}

TEST_F(Hil, ShellSetsParametersAndRejectsGarbage) {
    boot();
    run_ms(60);
    uart("set vref 9.5\r\n");
    uart("set ilim 99\n");
    uart("frobnicate\n");
    run_ms(60);
    std::string out = uart_text();
    EXPECT_NE(out.find("OK vref=9.500"), std::string::npos);
    EXPECT_NE(out.find("ERR out of range"), std::string::npos);
    EXPECT_NE(out.find("ERR unknown command 'frobnicate'"), std::string::npos);
    EXPECT_NEAR(vout(), 9.5, 0.1);
}

TEST_F(Hil, CorruptedShellLineIsNeverExecuted) {
    boot();
    run_ms(60);
    sim_fault_t noise{SIM_FAULT_UART_NOISE, 1.0, 0};
    board_inject_fault(b(), &noise);
    uart("set vref 5\n");
    run_ms(5);
    board_clear_fault(b(), &noise);
    uart("\n");
    run_ms(10);
    EXPECT_NEAR(st()->vref_target, 12.0f, 1e-3);
    EXPECT_NE(uart_text().find("ERR line discarded"), std::string::npos);
}

TEST_F(Hil, OverlongLineIsDiscarded) {
    boot();
    run_ms(20);
    std::string longline(FW_SHELL_LINE_MAX + 20, 'x');
    longline += "\n";
    uart(longline.c_str());
    run_ms(30);
    EXPECT_NE(uart_text().find("ERR line discarded"), std::string::npos);
}
