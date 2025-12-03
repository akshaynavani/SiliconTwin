// Config loader, scenario parser and end-to-end runner.
#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <string>

extern "C" {
#include "fw/can_proto.h"
#include "sim/config.h"
#include "sim/runner.h"
#include "sim/scenario.h"
}

#ifndef SILICONTWIN_SOURCE_DIR
#define SILICONTWIN_SOURCE_DIR "."
#endif

// ----------------------------------------------------------------- config
TEST(Config, ParsesSectionsUnitsAndComments) {
    sim_config_t cfg;
    sim_config_defaults(&cfg);
    char err[128] = "";
    const char *ini =
        "# comment\n"
        "[sim]\nname = demo ; trailing comment\nduration_ms = 321\n"
        "[plant]\ninductance_uh = 33\nvin=36\n"
        "[bench]\nenable = false\n"
        "[firmware]\nvref = 5.5\nmax_retries = 7\n";
    ASSERT_TRUE(sim_config_load_string(&cfg, ini, err, sizeof(err))) << err;
    EXPECT_STREQ(cfg.name, "demo");
    EXPECT_EQ(cfg.duration_ms, 321u);
    EXPECT_NEAR(cfg.board.plant.inductance, 33e-6, 1e-12);
    EXPECT_DOUBLE_EQ(cfg.board.plant.vin, 36.0);
    EXPECT_FALSE(cfg.enable_at_start);
    EXPECT_FLOAT_EQ(cfg.fw.vref, 5.5f);
    EXPECT_EQ(cfg.fw.max_retries, 7);
}

TEST(Config, RejectsUnknownKeysAndBadValues) {
    sim_config_t cfg;
    sim_config_defaults(&cfg);
    char err[128] = "";
    EXPECT_FALSE(sim_config_load_string(&cfg, "[plant]\nwarp = 9\n", err, sizeof(err)));
    EXPECT_NE(std::string(err).find("warp"), std::string::npos);
    EXPECT_FALSE(sim_config_load_string(&cfg, "[plant]\nvin = lots\n", err, sizeof(err)));
    EXPECT_FALSE(sim_config_load_string(&cfg, "[bench]\nenable = maybe\n", err, sizeof(err)));
    EXPECT_FALSE(sim_config_load_string(&cfg, "[sim\n", err, sizeof(err)));
    EXPECT_FALSE(sim_config_load_string(&cfg, "[sim]\njunk\n", err, sizeof(err)));
    EXPECT_FALSE(sim_config_load_string(&cfg, "[sim]\nduration_ms = -4\n", err, sizeof(err)));
}

TEST(Config, DottedOverrides) {
    sim_config_t cfg;
    sim_config_defaults(&cfg);
    char err[128] = "";
    EXPECT_TRUE(sim_config_apply_override(&cfg, "firmware.kp_v=1.25", err, sizeof(err)));
    EXPECT_FLOAT_EQ(cfg.fw.kp_v, 1.25f);
    EXPECT_TRUE(sim_config_apply_override(&cfg, "sim.seed=99", err, sizeof(err)));
    EXPECT_EQ(cfg.board.seed, 99u);
    EXPECT_FALSE(sim_config_apply_override(&cfg, "kp_v=1", err, sizeof(err)));
    EXPECT_FALSE(sim_config_apply_override(&cfg, "firmware.kp_v", err, sizeof(err)));
}

TEST(Config, ShippedConfigsLoad) {
    const char *names[] = {"nominal_12v", "low_voltage_5v", "high_output_24v",
                           "aggressive_gains", "defaults_fallback"};
    for (const char *n : names) {
        sim_config_t cfg;
        sim_config_defaults(&cfg);
        char err[256] = "";
        std::string path = std::string(SILICONTWIN_SOURCE_DIR) + "/configs/" + n + ".ini";
        EXPECT_TRUE(sim_config_load_file(&cfg, path.c_str(), err, sizeof(err))) << n << ": " << err;
    }
}

// --------------------------------------------------------------- scenario
TEST(Scenario, ParsesAllCommandKindsAndSortsByTime) {
    sim_scenario_t *sc = static_cast<sim_scenario_t *>(std::malloc(sizeof(sim_scenario_t)));
    scenario_init(sc);
    char err[200] = "";
    const char *text =
        "100 load 1.5\n"
        "10  enable 1   # inline comment\n"
        "20  uart \"set vref 5 # not a comment\"\n"
        "30  can 0x201 01 88 13\n"
        "40  can_cmd ilim 4.5\n"
        "50  fault adc_stuck 0 0\n"
        "60  clear adc_stuck 0\n"
        "70  mark window\n"
        "80  estop 1\n"
        "90  vin 30\n"
        "500 end\n";
    ASSERT_TRUE(scenario_parse_string(sc, text, err, sizeof(err))) << err;
    ASSERT_EQ(sc->count, 11u);
    EXPECT_EQ(sc->events[0].kind, EV_ENABLE);
    EXPECT_EQ(sc->events[0].time_us, 10000u);
    EXPECT_EQ(sc->events[1].kind, EV_UART);
    EXPECT_STREQ(sc->events[1].text, "set vref 5 # not a comment\n");
    EXPECT_EQ(sc->events[2].frame.id, 0x201);
    EXPECT_EQ(sc->events[2].frame.dlc, 3);
    EXPECT_EQ(sc->events[2].frame.data[1], 0x88);
    EXPECT_EQ(sc->events[3].frame.data[0], FW_CMD_SET_CURRENT_LIMIT);
    EXPECT_EQ(fw_get_u16(&sc->events[3].frame.data[1]), 4500);
    EXPECT_EQ(sc->events[4].fault.kind, SIM_FAULT_ADC_STUCK);
    EXPECT_EQ(sc->events[5].kind, EV_CLEAR);
    EXPECT_STREQ(sc->events[6].text, "window");
    EXPECT_EQ(sc->events[9].kind, EV_LOAD);
    EXPECT_TRUE(sc->has_end);
    EXPECT_EQ(sc->end_us, 500000u);
    std::free(sc);
}

TEST(Scenario, ReportsLineNumbersOnErrors) {
    sim_scenario_t *sc = static_cast<sim_scenario_t *>(std::malloc(sizeof(sim_scenario_t)));
    const char *bad[] = {"10 enable\n", "x enable 1\n", "10 teleport 3\n", "10 fault gremlins\n",
                         "10 can 0x900\n", "10 can 0x100 1 2 3 4 5 6 7 8 9\n", "10 load 0\n",
                         "10 can_cmd vref\n", "10 mark\n"};
    for (const char *line : bad) {
        scenario_init(sc);
        std::string text = std::string("# header\n") + line;
        char err[200] = "";
        EXPECT_FALSE(scenario_parse_string(sc, text.c_str(), err, sizeof(err))) << line;
        EXPECT_EQ(std::string(err).rfind("line 2:", 0), 0u) << err;
    }
    std::free(sc);
}

TEST(Scenario, ShippedScenariosParse) {
    const char *names[] = {"startup", "load_step", "short_circuit", "watchdog_hang",
                           "watchdog_reset_loop", "can_commands", "can_bus_errors",
                           "uart_shell", "estop", "vin_sag", "sensor_stuck", "overtemp"};
    sim_scenario_t *sc = static_cast<sim_scenario_t *>(std::malloc(sizeof(sim_scenario_t)));
    for (const char *n : names) {
        scenario_init(sc);
        char err[256] = "";
        std::string path = std::string(SILICONTWIN_SOURCE_DIR) + "/scenarios/" + n + ".scn";
        EXPECT_TRUE(scenario_load_file(sc, path.c_str(), err, sizeof(err))) << n << ": " << err;
        EXPECT_TRUE(sc->has_end) << n;
    }
    std::free(sc);
}

// ----------------------------------------------------------------- runner
class RunnerTest : public ::testing::Test {
protected:
    void SetUp() override {
        sim_config_defaults(&cfg_);
        sc_ = static_cast<sim_scenario_t *>(std::malloc(sizeof(sim_scenario_t)));
        m_ = static_cast<sim_metrics_t *>(std::calloc(1, sizeof(sim_metrics_t)));
        scenario_init(sc_);
    }
    void TearDown() override {
        sim_metrics_free(m_);
        std::free(m_);
        std::free(sc_);
    }
    bool run(const char *script) {
        char err[200] = "";
        if (!scenario_parse_string(sc_, script, err, sizeof(err))) return false;
        return sim_run(&cfg_, sc_, m_, nullptr, err, sizeof(err));
    }
    sim_config_t cfg_{};
    sim_scenario_t *sc_ = nullptr;
    sim_metrics_t *m_ = nullptr;
};

TEST_F(RunnerTest, StartupMetrics) {
    ASSERT_TRUE(run("0 mark startup\n100 end\n"));
    EXPECT_DOUBLE_EQ(m_->sim_time_ms, 100.0);
    EXPECT_EQ(m_->final_state, FW_STATE_REGULATING);
    EXPECT_GT(m_->startup_ms, 15.0);
    EXPECT_LT(m_->startup_ms, 40.0);
    EXPECT_LT(m_->overshoot_pct, 3.0);
    EXPECT_LT(m_->steady_state_error, 0.05);
    ASSERT_EQ(m_->n_windows, 1u);
    EXPECT_GT(m_->windows[0].settle_ms, 0.0);
    EXPECT_GT(m_->rx_telemetry, 5u);
    EXPECT_EQ(m_->rx_boot, 1u);
    EXPECT_EQ(m_->n_resets, 0u);
    ASSERT_NE(m_->uart_output, nullptr);
}

TEST_F(RunnerTest, RecordsFaultsResetsAndAcks) {
    ASSERT_TRUE(run("50 fault fw_hang 0\n150 can_cmd vref 10\n150 can_cmd vref 40\n250 end\n"));
    ASSERT_EQ(m_->n_resets, 1u);
    EXPECT_EQ(m_->resets[0].cause, HAL_RESET_WATCHDOG);
    EXPECT_EQ(m_->wdt_expirations, 1u);
    ASSERT_EQ(m_->n_acks, 2u);
    EXPECT_EQ(m_->acks[0].status, FW_ACK_OK);
    EXPECT_EQ(m_->acks[1].status, FW_ACK_OUT_OF_RANGE);
    EXPECT_NEAR(m_->vout_final, 10.0, 0.1);
}

TEST_F(RunnerTest, SameSeedIsBitForBitReproducible) {
    ASSERT_TRUE(run("20 fault can_errors 0.2\n150 end\n"));
    double v1 = m_->vout_final;
    uint32_t err1 = m_->can_error_frames;
    sim_metrics_free(m_);
    scenario_init(sc_);
    ASSERT_TRUE(run("20 fault can_errors 0.2\n150 end\n"));
    EXPECT_EQ(m_->vout_final, v1);
    EXPECT_EQ(m_->can_error_frames, err1);
}

TEST_F(RunnerTest, JsonOutputIsWellFormedEnoughToContainKeys) {
    ASSERT_TRUE(run("10 uart \"say \\\"hi\\\"\"\n60 end\n"));
    FILE *f = std::tmpfile();
    ASSERT_NE(f, nullptr);
    sim_metrics_write_json(m_, f);
    std::rewind(f);
    std::string json;
    char buf[512];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) json.append(buf, n);
    std::fclose(f);
    for (const char *key : {"\"regulation\"", "\"firmware\"", "\"faults\"", "\"windows\"",
                            "\"resets\"", "\"watchdog\"", "\"can\"", "\"uart\""}) {
        EXPECT_NE(json.find(key), std::string::npos) << key;
    }
    EXPECT_NE(json.find("unknown command"), std::string::npos);
    EXPECT_EQ(json.front(), '{');
}
