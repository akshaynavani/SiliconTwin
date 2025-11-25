// Unit tests for the basic simulated peripherals.
#include <gtest/gtest.h>

#include <cstring>
#include <string>

extern "C" {
#include "sim/adc.h"
#include "sim/clock.h"
#include "sim/gpio.h"
#include "sim/pwm.h"
#include "sim/rng.h"
#include "sim/uart.h"
#include "sim/watchdog.h"
}

// ------------------------------------------------------------------ clock
TEST(Clock, PeriodicFiresOnScheduleWithoutDrift) {
    sim_periodic_t p;
    sim_periodic_start(&p, 0, 100);
    int fired = 0;
    for (sim_time_us t = 10; t <= 1000; t += 10) {
        if (sim_periodic_due(&p, t)) fired++;
    }
    EXPECT_EQ(fired, 10);
}

TEST(Clock, PeriodicSkipsMissedDeadlines) {
    sim_periodic_t p;
    sim_periodic_start(&p, 0, 100);
    EXPECT_TRUE(sim_periodic_due(&p, 1000));
    EXPECT_FALSE(sim_periodic_due(&p, 1050));
    EXPECT_TRUE(sim_periodic_due(&p, 1100));
}

TEST(Clock, StoppedPeriodicNeverFires) {
    sim_periodic_t p;
    sim_periodic_start(&p, 0, 100);
    sim_periodic_stop(&p);
    EXPECT_FALSE(sim_periodic_due(&p, 10000));
}

// -------------------------------------------------------------------- rng
TEST(Rng, SameSeedSameSequence) {
    sim_rng_t a, b;
    sim_rng_seed(&a, 42);
    sim_rng_seed(&b, 42);
    for (int i = 0; i < 100; i++) EXPECT_EQ(sim_rng_next(&a), sim_rng_next(&b));
}

TEST(Rng, UniformInRangeAndGaussianRoughlyStandard) {
    sim_rng_t r;
    sim_rng_seed(&r, 7);
    double sum = 0, sq = 0;
    const int n = 20000;
    for (int i = 0; i < n; i++) {
        double u = sim_rng_uniform(&r);
        ASSERT_GE(u, 0.0);
        ASSERT_LT(u, 1.0);
        double g = sim_rng_gaussian(&r);
        sum += g;
        sq += g * g;
    }
    EXPECT_NEAR(sum / n, 0.0, 0.05);
    EXPECT_NEAR(sq / n, 1.0, 0.05);
}

TEST(Rng, ChanceExtremes) {
    sim_rng_t r;
    sim_rng_seed(&r, 1);
    EXPECT_FALSE(sim_rng_chance(&r, 0.0));
    EXPECT_TRUE(sim_rng_chance(&r, 1.0));
}

// ------------------------------------------------------------------- gpio
namespace {
struct IrqLog {
    int count = 0;
    uint8_t pin = 0xFF;
    bool level = false;
};
void record_irq(void *ctx, uint8_t pin, bool level) {
    auto *log = static_cast<IrqLog *>(ctx);
    log->count++;
    log->pin = pin;
    log->level = level;
}
}  // namespace

TEST(Gpio, PullResistorsDefineFloatingInputs) {
    gpio_port_t g;
    gpio_init(&g);
    gpio_configure(&g, 0, GPIO_DIR_INPUT, GPIO_PULL_UP);
    gpio_configure(&g, 1, GPIO_DIR_INPUT, GPIO_PULL_DOWN);
    EXPECT_TRUE(gpio_read(&g, 0));
    EXPECT_FALSE(gpio_read(&g, 1));
}

TEST(Gpio, ExternalDriveOverridesPull) {
    gpio_port_t g;
    gpio_init(&g);
    gpio_configure(&g, 3, GPIO_DIR_INPUT, GPIO_PULL_UP);
    gpio_drive_external(&g, 3, false);
    EXPECT_FALSE(gpio_read(&g, 3));
    gpio_release_external(&g, 3);
    EXPECT_TRUE(gpio_read(&g, 3));
}

TEST(Gpio, WriteOnlyAffectsOutputs) {
    gpio_port_t g;
    gpio_init(&g);
    EXPECT_FALSE(gpio_write(&g, 2, true));  // still an input
    gpio_configure(&g, 2, GPIO_DIR_OUTPUT, GPIO_PULL_NONE);
    EXPECT_TRUE(gpio_write(&g, 2, true));
    EXPECT_TRUE(gpio_read(&g, 2));
    EXPECT_TRUE(gpio_toggle(&g, 2));
    EXPECT_FALSE(gpio_read(&g, 2));
    EXPECT_FALSE(gpio_write(&g, GPIO_NUM_PINS, true));
}

TEST(Gpio, EdgeInterruptsFireOnConfiguredEdgeOnly) {
    gpio_port_t g;
    gpio_init(&g);
    IrqLog log;
    gpio_set_irq_handler(&g, record_irq, &log);
    gpio_irq_enable(&g, true);
    gpio_configure(&g, 5, GPIO_DIR_INPUT, GPIO_PULL_UP);
    gpio_set_edge(&g, 5, GPIO_EDGE_FALLING);

    gpio_drive_external(&g, 5, false);  // falling
    EXPECT_EQ(log.count, 1);
    EXPECT_EQ(log.pin, 5);
    EXPECT_FALSE(log.level);
    gpio_drive_external(&g, 5, true);  // rising: ignored
    EXPECT_EQ(log.count, 1);
    EXPECT_EQ(g.pins[5].edge_count, 1u);
}

TEST(Gpio, StuckAtFaultWinsOverEverything) {
    gpio_port_t g;
    gpio_init(&g);
    gpio_configure(&g, 4, GPIO_DIR_OUTPUT, GPIO_PULL_NONE);
    gpio_write(&g, 4, true);
    gpio_inject_stuck(&g, 4, false);
    EXPECT_FALSE(gpio_read(&g, 4));
    gpio_clear_stuck(&g, 4);
    EXPECT_TRUE(gpio_read(&g, 4));
}

TEST(Gpio, ResetRestoresInputsButKeepsExternalDrivers) {
    gpio_port_t g;
    gpio_init(&g);
    gpio_drive_external(&g, 0, true);
    gpio_configure(&g, 1, GPIO_DIR_OUTPUT, GPIO_PULL_NONE);
    gpio_write(&g, 1, true);
    gpio_reset(&g);
    EXPECT_EQ(g.pins[1].dir, GPIO_DIR_INPUT);
    EXPECT_FALSE(gpio_read(&g, 1));
    EXPECT_TRUE(gpio_read(&g, 0));
}

// ------------------------------------------------------------------- uart
namespace {
void run_uart(uart_t *u, sim_time_us &now, sim_time_us until) {
    for (; now < until; now += 10) uart_step(u, now);
}
}  // namespace

TEST(Uart, ByteTimeFollowsBaudRate) {
    uart_t u;
    uart_init(&u, 115200, nullptr);
    EXPECT_EQ(u.byte_time_us, 87u);
    uart_init(&u, 9600, nullptr);
    EXPECT_EQ(u.byte_time_us, 1042u);
}

TEST(Uart, TransmitIsPacedByLineRate) {
    uart_t u;
    uart_init(&u, 115200, nullptr);
    uart_enable(&u, true);
    const char *msg = "hello";
    EXPECT_EQ(uart_write(&u, reinterpret_cast<const uint8_t *>(msg), 5), 5u);
    sim_time_us now = 0;
    run_uart(&u, now, 200);  // ~2 byte times
    size_t len = 0;
    uart_host_output(&u, &len);
    EXPECT_GE(len, 2u);
    EXPECT_LE(len, 3u);
    run_uart(&u, now, 1000);
    EXPECT_STREQ(uart_host_output(&u, &len), "hello");
}

TEST(Uart, TxFifoIsBounded) {
    uart_t u;
    uart_init(&u, 115200, nullptr);
    uint8_t buf[100] = {0};
    EXPECT_EQ(uart_write(&u, buf, sizeof(buf)), static_cast<size_t>(UART_HW_FIFO_SIZE));
    EXPECT_EQ(uart_tx_free(&u), 0u);
}

TEST(Uart, ReceiveDeliversHostBytes) {
    uart_t u;
    uart_init(&u, 115200, nullptr);
    uart_enable(&u, true);
    uart_host_send(&u, "abc", 3);
    sim_time_us now = 0;
    run_uart(&u, now, 1000);
    char buf[8] = {0};
    EXPECT_EQ(uart_read(&u, reinterpret_cast<uint8_t *>(buf), sizeof(buf)), 3u);
    EXPECT_STREQ(buf, "abc");
}

TEST(Uart, OverrunWhenFirmwareDoesNotDrain) {
    uart_t u;
    uart_init(&u, 115200, nullptr);
    uart_enable(&u, true);
    std::string data(UART_HW_FIFO_SIZE + 10, 'x');
    uart_host_send(&u, data.c_str(), data.size());
    sim_time_us now = 0;
    run_uart(&u, now, 100000);
    EXPECT_TRUE(uart_take_overrun(&u));
    EXPECT_FALSE(uart_take_overrun(&u));  // flag is cleared on read
    EXPECT_EQ(u.stats.overruns, 10u);
    EXPECT_EQ(uart_rx_available(&u), static_cast<size_t>(UART_HW_FIFO_SIZE));
}

TEST(Uart, DisabledReceiverDropsBytes) {
    uart_t u;
    uart_init(&u, 115200, nullptr);
    uart_host_send(&u, "zz", 2);
    sim_time_us now = 0;
    run_uart(&u, now, 1000);
    EXPECT_EQ(uart_rx_available(&u), 0u);
}

TEST(Uart, NoiseCorruptsAndFlagsFramingErrors) {
    sim_rng_t rng;
    sim_rng_seed(&rng, 3);
    uart_t u;
    uart_init(&u, 115200, &rng);
    uart_enable(&u, true);
    uart_set_noise(&u, 1.0);
    uart_host_send(&u, "AAAA", 4);
    sim_time_us now = 0;
    run_uart(&u, now, 1000);
    uint8_t buf[4];
    ASSERT_EQ(uart_read(&u, buf, 4), 4u);
    for (uint8_t b : buf) EXPECT_NE(b, 'A');
    EXPECT_TRUE(uart_take_framing_error(&u));
    EXPECT_EQ(u.stats.framing_errors, 4u);
}

// -------------------------------------------------------------------- adc
TEST(Adc, QuantisesAndSaturates) {
    adc_t a;
    adc_init(&a, 2, 12, nullptr);
    adc_set_full_scale(&a, 0, 30.0);
    adc_set_input(&a, 0, 15.0);
    EXPECT_NEAR(adc_convert(&a, 0), 2048, 1);
    adc_set_input(&a, 0, 50.0);
    EXPECT_EQ(adc_convert(&a, 0), 4095);
    adc_set_input(&a, 0, -3.0);
    EXPECT_EQ(adc_convert(&a, 0), 0);
    EXPECT_EQ(adc_convert(&a, 7), 0);  // invalid channel
}

TEST(Adc, StuckAndOffsetFaults) {
    adc_t a;
    adc_init(&a, 1, 12, nullptr);
    adc_set_full_scale(&a, 0, 4095.0);
    adc_set_input(&a, 0, 1000.0);
    adc_inject_offset(&a, 0, 25.0);
    EXPECT_EQ(adc_convert(&a, 0), 1025);
    adc_inject_stuck(&a, 0, 7);
    EXPECT_EQ(adc_convert(&a, 0), 7);
    adc_clear_faults(&a, 0);
    EXPECT_EQ(adc_convert(&a, 0), 1000);
}

TEST(Adc, NoiseIsBoundedAndSeeded) {
    sim_rng_t rng;
    sim_rng_seed(&rng, 11);
    adc_t a;
    adc_init(&a, 1, 12, &rng);
    adc_set_full_scale(&a, 0, 4095.0);
    adc_set_input(&a, 0, 2000.0);
    adc_set_noise(&a, 2.0);
    double sum = 0;
    for (int i = 0; i < 2000; i++) {
        int c = adc_convert(&a, 0);
        EXPECT_NEAR(c, 2000, 15);
        sum += c;
    }
    EXPECT_NEAR(sum / 2000.0, 2000.0, 0.5);
}

// -------------------------------------------------------------------- pwm
TEST(Pwm, ClampsAndHonoursEnableAndBreak) {
    pwm_t p;
    pwm_init(&p, 100000, 0.01, 0.95);
    pwm_set_duty(&p, 0.5);
    EXPECT_EQ(pwm_effective_duty(&p), 0.0);  // disabled
    pwm_enable(&p, true);
    EXPECT_DOUBLE_EQ(pwm_effective_duty(&p), 0.5);
    pwm_set_duty(&p, 1.2);
    EXPECT_DOUBLE_EQ(pwm_effective_duty(&p), 0.95);
    pwm_set_duty(&p, 0.005);
    EXPECT_DOUBLE_EQ(pwm_effective_duty(&p), 0.0);  // below minimum on-time
    EXPECT_EQ(p.clamped_updates, 2u);
    pwm_set_duty(&p, 0.4);
    pwm_set_break(&p, true);
    EXPECT_EQ(pwm_effective_duty(&p), 0.0);
}

// --------------------------------------------------------------- watchdog
TEST(Watchdog, ExpiresWithoutKicks) {
    watchdog_t w;
    wdt_init(&w);
    wdt_start(&w, 0, 50000);
    EXPECT_FALSE(wdt_step(&w, 49999));
    EXPECT_TRUE(wdt_step(&w, 50000));
    EXPECT_FALSE(wdt_step(&w, 60000));  // fires once
    EXPECT_EQ(w.expirations, 1u);
}

TEST(Watchdog, KicksPostponeExpiryAndTrackWorstGap) {
    watchdog_t w;
    wdt_init(&w);
    wdt_start(&w, 0, 50000);
    for (sim_time_us t = 1000; t <= 200000; t += 1000) {
        wdt_kick(&w, t);
        ASSERT_FALSE(wdt_step(&w, t));
    }
    wdt_kick(&w, 230000);
    EXPECT_EQ(w.max_kick_interval, 30000u);
    EXPECT_EQ(w.kicks, 201u);
}

TEST(Watchdog, TimeoutIsWriteOnceUntilReset) {
    watchdog_t w;
    wdt_init(&w);
    wdt_start(&w, 0, 10000);
    wdt_start(&w, 0, 1000000);  // ignored
    EXPECT_TRUE(wdt_step(&w, 10000));
    wdt_reset(&w);
    EXPECT_FALSE(w.running);
}
