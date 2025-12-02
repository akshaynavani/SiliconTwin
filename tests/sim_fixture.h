// Shared fixture: a full board twin running the real firmware.
#ifndef SILICONTWIN_TESTS_SIM_FIXTURE_H
#define SILICONTWIN_TESTS_SIM_FIXTURE_H

#include <gtest/gtest.h>

#include <cstring>
#include <memory>
#include <string>
#include <vector>

extern "C" {
#include "fw/can_proto.h"
#include "fw/firmware.h"
#include "fw/params.h"
#include "fw/protection.h"
#include "sim/board.h"
}

class BoardFixture : public ::testing::Test {
protected:
    void SetUp() override {
        board_ = std::make_unique<sim_board_t>();
        board_default_config(&cfg_);
        fw_params_defaults(&params_);
    }

    sim_board_t *b() { return board_.get(); }
    const fw_status_t *st() { return fw_status(); }

    void boot(bool enable = true, bool flash = true, bool corrupt = false) {
        mcu_firmware_t fw{fw_init, fw_main_tick};
        board_init(b(), &cfg_, &fw);
        if (flash) {
            fw_params_t image = params_;
            fw_params_seal(&image);
            if (corrupt) image.crc ^= 0xFFFFu;
            mcu_flash_program(&b()->mcu, FW_PARAMS_FLASH_OFFSET, &image, sizeof(image));
        }
        board_set_enable(b(), enable);
        board_power_on(b());
        frames_.clear();
    }

    // Advances simulated time, collecting every frame the bench node sees.
    void run_ms(double ms) {
        sim_time_us end = board_now(b()) + static_cast<sim_time_us>(ms * 1000.0);
        while (board_now(b()) < end) {
            board_step(b());
            can_frame_t f;
            while (board_bench_can_receive(b(), &f)) frames_.push_back(f);
        }
    }

    // Runs until the predicate holds or the timeout expires; returns elapsed ms.
    template <typename Pred>
    double run_until(Pred pred, double timeout_ms) {
        sim_time_us start = board_now(b());
        sim_time_us end = start + static_cast<sim_time_us>(timeout_ms * 1000.0);
        while (board_now(b()) < end && !pred()) run_ms(0.1);
        return static_cast<double>(board_now(b()) - start) / 1000.0;
    }

    double vout() { return b()->plant.v_out; }

    void can_cmd(uint8_t cmd, uint16_t arg = 0, bool with_arg = false) {
        uint8_t data[3] = {cmd, static_cast<uint8_t>(arg & 0xFF), static_cast<uint8_t>(arg >> 8)};
        board_bench_can_send(b(), FW_CAN_ID_COMMAND + params_.can_node_id, data,
                             with_arg ? 3 : 1);
    }

    std::vector<can_frame_t> frames_with_base(uint16_t base) const {
        std::vector<can_frame_t> out;
        for (const auto &f : frames_) {
            if ((f.id & 0x780u) == base) out.push_back(f);
        }
        return out;
    }

    std::string uart_text() {
        size_t len = 0;
        return std::string(uart_host_output(&b()->mcu.uart, &len), len);
    }

    void uart(const char *text) { board_uart_send(b(), text); }

    std::unique_ptr<sim_board_t> board_;
    board_config_t cfg_{};
    fw_params_t params_{};
    std::vector<can_frame_t> frames_;
};

#endif  // SILICONTWIN_TESTS_SIM_FIXTURE_H
