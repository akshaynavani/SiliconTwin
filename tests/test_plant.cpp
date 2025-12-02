// Averaged buck converter plant model.
#include <gtest/gtest.h>

extern "C" {
#include "sim/plant.h"
}

namespace {
void run(plant_t *p, double duty, bool hiz, double seconds, double dt = 1e-6) {
    int n = static_cast<int>(seconds / dt);
    for (int i = 0; i < n; i++) plant_step(p, duty, hiz, dt);
}
}  // namespace

TEST(Plant, OpenLoopSteadyStateMatchesDividerEquation) {
    plant_params_t prm;
    plant_default_params(&prm);
    plant_t p;
    plant_init(&p, &prm);
    run(&p, 0.25, false, 0.05);
    // Vout = D*Vin * R/(R + R_L)
    double expected = 0.25 * prm.vin * prm.r_load / (prm.r_load + prm.r_inductor);
    EXPECT_NEAR(p.v_out, expected, 0.05);
    EXPECT_NEAR(p.i_l, expected / prm.r_load, 0.05);
}

TEST(Plant, ShortCircuitCollapsesOutput) {
    plant_params_t prm;
    plant_default_params(&prm);
    plant_t p;
    plant_init(&p, &prm);
    run(&p, 0.25, false, 0.02);
    plant_set_short(&p, true, 0.02);
    EXPECT_LT(plant_effective_load(&p), 0.02);
    run(&p, 0.02, false, 0.02);
    EXPECT_LT(p.v_out, 0.5);
    plant_set_short(&p, false, 0.0);
    EXPECT_DOUBLE_EQ(plant_effective_load(&p), prm.r_load);
}

TEST(Plant, HighImpedanceStageNeverReversesInductorCurrent) {
    plant_params_t prm;
    plant_default_params(&prm);
    plant_t p;
    plant_init(&p, &prm);
    run(&p, 0.25, false, 0.02);
    ASSERT_GT(p.i_l, 1.0);
    run(&p, 0.0, true, 0.05);
    EXPECT_GE(p.i_l, 0.0);
    EXPECT_LT(p.v_out, 1.0);  // output discharged into the load
}

TEST(Plant, ThermalModelApproachesAmbientPlusLossRise) {
    plant_params_t prm;
    plant_default_params(&prm);
    prm.tau_thermal = 0.01;  // speed the test up
    plant_t p;
    plant_init(&p, &prm);
    run(&p, 0.25, false, 0.1);
    double expected = prm.t_ambient + prm.r_thermal * p.p_loss;
    EXPECT_NEAR(p.temp, expected, 0.5);
    EXPECT_GT(p.temp, prm.t_ambient);
}

TEST(Plant, SettersValidateInputs) {
    plant_params_t prm;
    plant_default_params(&prm);
    plant_t p;
    plant_init(&p, &prm);
    plant_set_load(&p, -1.0);
    EXPECT_DOUBLE_EQ(p.r_load, prm.r_load);
    plant_set_vin(&p, -5.0);
    EXPECT_DOUBLE_EQ(p.p.vin, 0.0);
}
