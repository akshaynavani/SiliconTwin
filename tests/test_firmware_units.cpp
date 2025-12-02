// Pure-logic firmware modules tested in isolation (no simulator needed).
#include <gtest/gtest.h>

#include <cmath>
#include <cstring>

extern "C" {
#include "fw/can_proto.h"
#include "fw/control.h"
#include "fw/params.h"
#include "fw/protection.h"
#include "fw/shell.h"
}

// ----------------------------------------------------------------- params
TEST(Params, Crc32KnownVector) {
    EXPECT_EQ(fw_crc32("123456789", 9), 0xCBF43926u);
}

TEST(Params, DefaultsAreSealedValidAndSane) {
    fw_params_t p;
    fw_params_defaults(&p);
    EXPECT_TRUE(fw_params_valid(&p));
    EXPECT_TRUE(fw_params_sane(&p));
}

TEST(Params, AnyBitFlipInvalidatesTheImage) {
    fw_params_t p;
    fw_params_defaults(&p);
    auto *bytes = reinterpret_cast<uint8_t *>(&p);
    bytes[12] ^= 0x01;
    EXPECT_FALSE(fw_params_valid(&p));
    fw_params_seal(&p);
    EXPECT_TRUE(fw_params_valid(&p));
}

TEST(Params, SanityRejectsDangerousValues) {
    fw_params_t p;
    fw_params_defaults(&p);
    p.ovp_trip = p.vref - 1.0f;  // OVP below setpoint
    EXPECT_FALSE(fw_params_sane(&p));
    fw_params_defaults(&p);
    p.current_limit = p.ocp_trip + 1.0f;
    EXPECT_FALSE(fw_params_sane(&p));
    fw_params_defaults(&p);
    p.kp_v = NAN;
    EXPECT_FALSE(fw_params_sane(&p));
    fw_params_defaults(&p);
    p.wdt_timeout_ms = 1;
    EXPECT_FALSE(fw_params_sane(&p));
}

// ---------------------------------------------------------------- control
TEST(Pi, ProportionalAndIntegralAction) {
    fw_pi_t pi;
    fw_pi_init(&pi, 2.0f, 100.0f, 0.001f, -10.0f, 10.0f);
    float out = fw_pi_step(&pi, 1.0f);
    EXPECT_NEAR(out, 2.0f + 0.1f, 1e-5);
    out = fw_pi_step(&pi, 1.0f);
    EXPECT_NEAR(out, 2.0f + 0.2f, 1e-5);
}

TEST(Pi, AntiWindupStopsIntegratorInSaturation) {
    fw_pi_t pi;
    fw_pi_init(&pi, 1.0f, 1000.0f, 0.001f, 0.0f, 5.0f);
    for (int i = 0; i < 1000; i++) fw_pi_step(&pi, 10.0f);
    EXPECT_TRUE(pi.saturated);
    EXPECT_LE(pi.integ, 5.0f);
    // Recovers immediately once the error reverses: no wound-up integrator.
    float out = 0;
    for (int i = 0; i < 5; i++) out = fw_pi_step(&pi, -1.0f);
    EXPECT_LT(out, 5.0f);
}

TEST(Control, CurrentReferenceIsClampedToLimit) {
    fw_ctrl_t c;
    fw_ctrl_init(&c, 3.0f, 1500.0f, 0.45f, 8.0f, 1e-4f);
    fw_meas_t m{0.0f, 0.0f, 48.0f, 25.0f};
    fw_ctrl_step(&c, 12.0f, &m);
    EXPECT_FLOAT_EQ(c.i_ref, 8.0f);
    fw_ctrl_set_current_limit(&c, 4.0f);
    fw_ctrl_step(&c, 12.0f, &m);
    EXPECT_FLOAT_EQ(c.i_ref, 4.0f);
}

TEST(Control, DutyUsesFeedForwardAndIsClamped) {
    fw_ctrl_t c;
    fw_ctrl_init(&c, 3.0f, 0.0f, 0.45f, 8.0f, 1e-4f);
    // At the setpoint with matching current the duty is ~ Vout/Vin.
    fw_meas_t m{12.0f, 0.0f, 48.0f, 25.0f};
    float d = fw_ctrl_step(&c, 12.0f, &m);
    EXPECT_NEAR(d, 0.25f, 0.01f);
    // Collapsed input voltage: duty saturates at the hardware maximum.
    m.vin = 5.0f;
    d = fw_ctrl_step(&c, 12.0f, &m);
    EXPECT_FLOAT_EQ(d, 0.95f);
    m.vin = 0.0f;
    EXPECT_FLOAT_EQ(fw_ctrl_step(&c, 12.0f, &m), 0.0f);
}

TEST(Ramp, ReachesTargetInRequestedSteps) {
    fw_ramp_t r;
    fw_ramp_start(&r, 0.0f, 10.0f, 100);
    int steps = 0;
    while (!fw_ramp_done(&r)) {
        fw_ramp_step(&r);
        steps++;
        ASSERT_LE(steps, 101);
    }
    EXPECT_FLOAT_EQ(r.value, 10.0f);
    fw_ramp_start(&r, 5.0f, 1.0f, 0);
    EXPECT_TRUE(fw_ramp_done(&r));
    EXPECT_FLOAT_EQ(r.value, 1.0f);
}

// ------------------------------------------------------------- protection
class ProtectionTest : public ::testing::Test {
protected:
    void SetUp() override {
        fw_params_defaults(&params_);
        fw_prot_init(&prot_, &params_);
    }
    fw_params_t params_{};
    fw_prot_t prot_{};
};

TEST_F(ProtectionTest, FastChecksTripOnSingleSample) {
    fw_meas_t m{12.0f, 5.0f, 48.0f, 25.0f};
    EXPECT_EQ(fw_prot_fast(&prot_, &m), 0u);
    m.iout = 13.0f;
    EXPECT_EQ(fw_prot_fast(&prot_, &m), FW_FAULT_OCP);
    m.iout = 5.0f;
    m.vout = 15.0f;
    EXPECT_EQ(fw_prot_fast(&prot_, &m), FW_FAULT_OVP);
}

TEST_F(ProtectionTest, UnderVoltageIsDebouncedAndOnlyWhileRegulating) {
    fw_meas_t m{5.0f, 5.0f, 48.0f, 25.0f};
    float duty = 5.0f / 48.0f;
    for (unsigned i = 0; i < FW_PROT_UVP_MS * 2; i++) {
        EXPECT_EQ(fw_prot_slow(&prot_, &m, duty, 12.0f, false) & FW_FAULT_UVP, 0u);
    }
    uint8_t f = 0;
    unsigned ticks = 0;
    while (!(f & FW_FAULT_UVP) && ticks < 100) {
        f = fw_prot_slow(&prot_, &m, duty, 12.0f, true);
        ticks++;
    }
    EXPECT_EQ(ticks, FW_PROT_UVP_MS);
}

TEST_F(ProtectionTest, InputWindowAndTemperature) {
    fw_meas_t m{12.0f, 5.0f, 15.0f, 25.0f};
    uint8_t f = 0;
    for (unsigned i = 0; i < FW_PROT_VIN_MS; i++) f = fw_prot_slow(&prot_, &m, 0.8f, 12.0f, false);
    EXPECT_TRUE(f & FW_FAULT_VIN_UV);
    EXPECT_FALSE(fw_prot_condition_cleared(&prot_, FW_FAULT_VIN_UV, &m));
    m.vin = params_.vin_min + 0.5f;  // inside hysteresis band
    EXPECT_FALSE(fw_prot_condition_cleared(&prot_, FW_FAULT_VIN_UV, &m));
    m.vin = 48.0f;
    EXPECT_TRUE(fw_prot_condition_cleared(&prot_, FW_FAULT_VIN_UV, &m));

    m.temp = 120.0f;
    EXPECT_TRUE(fw_prot_start_inhibit(&prot_, &m) & FW_FAULT_OTP);
    m.temp = 95.0f;  // below trip but above clear: still inhibited
    EXPECT_TRUE(fw_prot_start_inhibit(&prot_, &m) & FW_FAULT_OTP);
    EXPECT_FALSE(fw_prot_condition_cleared(&prot_, FW_FAULT_OTP, &m));
}

TEST_F(ProtectionTest, SensorPlausibilityDetectsDisagreement) {
    // Measured output says 4 V but the switch node averages 12 V.
    fw_meas_t m{4.0f, 5.0f, 48.0f, 25.0f};
    float duty = 12.25f / 48.0f;
    uint8_t f = 0;
    for (int i = 0; i < 20 && !(f & FW_FAULT_SENSOR); i++) {
        f = fw_prot_slow(&prot_, &m, duty, 4.0f, true);
    }
    EXPECT_TRUE(f & FW_FAULT_SENSOR);
}

TEST_F(ProtectionTest, SensorPinnedAtRailIsAFault) {
    fw_meas_t m{0.0f, 0.5f, 48.0f, 25.0f};
    uint8_t f = 0;
    for (unsigned i = 0; i < FW_PROT_SENSOR_MS; i++) f = fw_prot_slow(&prot_, &m, 0.0f, 0.5f, true);
    EXPECT_TRUE(f & FW_FAULT_SENSOR);
}

TEST(ProtectionPolicy, RecoveryClassification) {
    EXPECT_EQ(fw_prot_recovery(FW_FAULT_OCP), FW_RECOVERY_RETRY);
    EXPECT_EQ(fw_prot_recovery(FW_FAULT_UVP), FW_RECOVERY_RETRY);
    EXPECT_EQ(fw_prot_recovery(FW_FAULT_VIN_UV), FW_RECOVERY_WAIT_CLEAR);
    EXPECT_EQ(fw_prot_recovery(FW_FAULT_OTP), FW_RECOVERY_WAIT_CLEAR);
    EXPECT_EQ(fw_prot_recovery(FW_FAULT_SENSOR), FW_RECOVERY_LATCH);
    EXPECT_EQ(fw_prot_recovery(FW_FAULT_WATCHDOG), FW_RECOVERY_LATCH);
    EXPECT_STREQ(fw_fault_name(FW_FAULT_OCP), "OCP");
    EXPECT_STREQ(fw_fault_name(FW_FAULT_OCP | FW_FAULT_OVP), "MULTI");
}

// --------------------------------------------------------------- CAN codec
TEST(CanProto, TelemetryRoundTrip) {
    fw_telem_t t{12003, -1500, 480, 2, 0};
    hal_can_frame_t f;
    fw_encode_telem(&f, 1, &t);
    EXPECT_EQ(f.id, FW_CAN_ID_TELEM + 1);
    EXPECT_EQ(f.dlc, 8);
    fw_telem_t back{};
    ASSERT_TRUE(fw_decode_telem(&f, &back));
    EXPECT_EQ(back.vout_mv, 12003);
    EXPECT_EQ(back.iout_ma, -1500);
    EXPECT_EQ(back.vin_dv, 480);
    EXPECT_EQ(back.state, 2);
    f.id = FW_CAN_ID_STATUS + 1;
    EXPECT_FALSE(fw_decode_telem(&f, &back));
}

TEST(CanProto, FixedPointConversionsSaturate) {
    EXPECT_EQ(fw_to_u16(12.0f, 1000.0f), 12000);
    EXPECT_EQ(fw_to_u16(-3.0f, 1000.0f), 0);
    EXPECT_EQ(fw_to_u16(100.0f, 1000.0f), 65535);
    EXPECT_EQ(fw_to_i16(-2.5f, 1000.0f), -2500);
    EXPECT_EQ(fw_to_i16(50.0f, 1000.0f), 32767);
    EXPECT_EQ(fw_to_i16(NAN, 1000.0f), 0);
    uint8_t buf[2];
    fw_put_u16(buf, 0xBEEF);
    EXPECT_EQ(buf[0], 0xEF);
    EXPECT_EQ(fw_get_u16(buf), 0xBEEF);
}

// ----------------------------------------------------------- shell parsing
TEST(Shell, TokenizerSplitsOnWhitespace) {
    char line[] = "  set   vref\t9.5  ";
    char *argv[FW_SHELL_MAX_ARGS];
    int argc = fw_shell_tokenize(line, argv, FW_SHELL_MAX_ARGS);
    ASSERT_EQ(argc, 3);
    EXPECT_STREQ(argv[0], "set");
    EXPECT_STREQ(argv[1], "vref");
    EXPECT_STREQ(argv[2], "9.5");
}

TEST(Shell, TokenizerRespectsArgumentLimit) {
    char line[] = "a b c d e f g h";
    char *argv[3];
    EXPECT_EQ(fw_shell_tokenize(line, argv, 3), 3);
}
