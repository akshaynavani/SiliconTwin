// CAN bus model: arbitration, filtering, fault confinement.
#include <gtest/gtest.h>

extern "C" {
#include "sim/can.h"
}

namespace {

class CanBusTest : public ::testing::Test {
protected:
    void SetUp() override {
        sim_rng_seed(&rng_, 5);
        can_bus_init(&bus_, 500000, &rng_);
        a_ = static_cast<uint8_t>(can_bus_attach(&bus_, "a"));
        b_ = static_cast<uint8_t>(can_bus_attach(&bus_, "b"));
        c_ = static_cast<uint8_t>(can_bus_attach(&bus_, "c"));
    }

    void run_us(sim_time_us duration) {
        sim_time_us end = now_ + duration;
        for (; now_ < end; now_ += 10) can_bus_step(&bus_, now_);
    }

    static can_frame_t frame(uint16_t id, uint8_t dlc = 2) {
        can_frame_t f{};
        f.id = id;
        f.dlc = dlc;
        for (uint8_t i = 0; i < dlc; i++) f.data[i] = static_cast<uint8_t>(id + i);
        return f;
    }

    sim_rng_t rng_{};
    can_bus_t bus_{};
    uint8_t a_ = 0, b_ = 0, c_ = 0;
    sim_time_us now_ = 0;
};

}  // namespace

TEST_F(CanBusTest, FrameDurationScalesWithPayloadAndBitrate) {
    sim_time_us empty = can_frame_duration_us(&bus_, 0);
    sim_time_us full = can_frame_duration_us(&bus_, 8);
    EXPECT_GT(full, empty);
    EXPECT_NEAR(static_cast<double>(full), 250.0, 30.0);  // ~125 bits at 500 kbit/s
    bus_.bitrate = 125000;
    EXPECT_NEAR(static_cast<double>(can_frame_duration_us(&bus_, 8)), 4.0 * full, 8.0);
}

TEST_F(CanBusTest, DeliversToAllOtherNodes) {
    can_frame_t f = frame(0x123);
    ASSERT_TRUE(can_node_send(&bus_, a_, &f));
    run_us(1000);
    can_frame_t r{};
    EXPECT_TRUE(can_node_receive(&bus_, b_, &r));
    EXPECT_EQ(r.id, 0x123);
    EXPECT_EQ(r.data[1], static_cast<uint8_t>(0x124));
    EXPECT_TRUE(can_node_receive(&bus_, c_, &r));
    EXPECT_FALSE(can_node_receive(&bus_, a_, &r));  // no self-reception
    EXPECT_EQ(bus_.frames_ok, 1u);
}

TEST_F(CanBusTest, LowestIdentifierWinsArbitration) {
    can_frame_t hi = frame(0x300), lo = frame(0x050);
    can_node_send(&bus_, a_, &hi);
    can_node_send(&bus_, b_, &lo);
    run_us(2000);
    ASSERT_EQ(can_bus_log_count(&bus_), 2u);
    EXPECT_EQ(can_bus_log_at(&bus_, 0)->frame.id, 0x050);
    EXPECT_EQ(can_bus_log_at(&bus_, 1)->frame.id, 0x300);
    EXPECT_EQ(bus_.nodes[a_].stats.arbitration_lost, 1u);
}

TEST_F(CanBusTest, AcceptanceFiltersDropUnwantedIds) {
    ASSERT_TRUE(can_node_set_filter(&bus_, b_, 0, 0x200, 0x7F0));
    can_frame_t wanted = frame(0x205), other = frame(0x305);
    can_node_send(&bus_, a_, &wanted);
    can_node_send(&bus_, a_, &other);
    run_us(2000);
    can_frame_t r{};
    ASSERT_TRUE(can_node_receive(&bus_, b_, &r));
    EXPECT_EQ(r.id, 0x205);
    EXPECT_FALSE(can_node_receive(&bus_, b_, &r));
    EXPECT_EQ(bus_.nodes[b_].stats.rx_filtered, 1u);
}

TEST_F(CanBusTest, RxFifoOverrunIsCounted) {
    for (int i = 0; i < CAN_RX_FIFO_DEPTH + 4; i++) {
        can_frame_t f = frame(0x100);
        while (!can_node_send(&bus_, a_, &f)) run_us(100);
    }
    run_us(20000);
    EXPECT_EQ(bus_.nodes[b_].stats.rx_overruns, 4u);
}

TEST_F(CanBusTest, TxQueueRejectsWhenFull) {
    can_frame_t f = frame(0x10);
    for (int i = 0; i < CAN_TX_QUEUE_DEPTH; i++) ASSERT_TRUE(can_node_send(&bus_, a_, &f));
    EXPECT_FALSE(can_node_send(&bus_, a_, &f));
    EXPECT_EQ(bus_.nodes[a_].stats.tx_dropped, 1u);
    can_frame_t bad = frame(0x10);
    bad.dlc = 9;
    EXPECT_FALSE(can_node_send(&bus_, b_, &bad));
}

TEST_F(CanBusTest, BusErrorsRaiseErrorCountersAndRetransmit) {
    can_bus_set_error_rate(&bus_, 1.0);
    can_frame_t f = frame(0x42);
    can_node_send(&bus_, a_, &f);
    run_us(1000);  // a handful of failed attempts, well short of bus-off
    EXPECT_NE(can_node_state(&bus_, a_), CAN_STATE_BUS_OFF);
    EXPECT_GT(bus_.nodes[a_].tec, 0u);
    EXPECT_GT(bus_.nodes[b_].rec, 0u);
    EXPECT_EQ(bus_.frames_ok, 0u);

    can_bus_set_error_rate(&bus_, 0.0);
    run_us(5000);
    EXPECT_EQ(bus_.frames_ok, 1u);  // automatic retransmission succeeded
}

TEST_F(CanBusTest, PersistentErrorsEndInBusOffAndRecoveryRestoresNode) {
    can_bus_set_error_rate(&bus_, 1.0);
    can_frame_t f = frame(0x42);
    can_node_send(&bus_, a_, &f);
    run_us(50000);
    EXPECT_EQ(can_node_state(&bus_, a_), CAN_STATE_BUS_OFF);
    EXPECT_EQ(bus_.nodes[a_].stats.bus_off_events, 1u);
    EXPECT_FALSE(can_node_send(&bus_, a_, &f));

    can_bus_set_error_rate(&bus_, 0.0);
    can_node_request_recovery(&bus_, a_, now_);
    run_us(1000);
    EXPECT_EQ(can_node_state(&bus_, a_), CAN_STATE_BUS_OFF);  // 128 x 11 bits = 2.8 ms
    run_us(3000);
    EXPECT_EQ(can_node_state(&bus_, a_), CAN_STATE_ERROR_ACTIVE);
    EXPECT_EQ(bus_.nodes[a_].tec, 0u);
    EXPECT_TRUE(can_node_send(&bus_, a_, &f));
}

TEST_F(CanBusTest, LoneNodeParksInErrorPassiveOnMissingAck) {
    can_node_enable(&bus_, b_, false);
    can_node_enable(&bus_, c_, false);
    can_frame_t f = frame(0x77);
    can_node_send(&bus_, a_, &f);
    run_us(100000);
    EXPECT_EQ(can_node_state(&bus_, a_), CAN_STATE_ERROR_PASSIVE);
    EXPECT_LT(bus_.nodes[a_].tec, 256u);
}

TEST_F(CanBusTest, ForcedBusOffAbortsPendingFrames) {
    can_frame_t f = frame(0x55);
    can_node_send(&bus_, a_, &f);
    can_node_force_bus_off(&bus_, a_);
    EXPECT_EQ(can_node_state(&bus_, a_), CAN_STATE_BUS_OFF);
    EXPECT_EQ(bus_.nodes[a_].tx_count, 0u);
    run_us(2000);
    EXPECT_EQ(bus_.frames_ok, 0u);
}

TEST_F(CanBusTest, ResetNodeIsDisabledUntilReEnabled) {
    can_node_reset(&bus_, b_);
    can_frame_t f = frame(0x60);
    EXPECT_FALSE(can_node_send(&bus_, b_, &f));
    can_node_enable(&bus_, b_, true);
    EXPECT_TRUE(can_node_send(&bus_, b_, &f));
}

TEST(CanState, Names) {
    EXPECT_STREQ(can_state_name(CAN_STATE_ERROR_ACTIVE), "error_active");
    EXPECT_STREQ(can_state_name(CAN_STATE_BUS_OFF), "bus_off");
}
