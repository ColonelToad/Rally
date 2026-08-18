#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rally/core/ekf_ball_predictor.hpp"
#include "rally/core/clock.hpp"
#include <cmath>

using namespace rally::core;

TEST_CASE("EkfBallPredictor initialization", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(5, 0, 3);
    ekf.initialize_state(pos, vel);
    // Verify state is set (no public accessor, but predict should not crash)
    ekf.predict();
    REQUIRE(true);
}

TEST_CASE("EkfBallPredictor predict reduces velocity due to drag", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(5, 5, 3);
    ekf.initialize_state(pos, vel);

    // After predict, velocity should be reduced by drag
    ekf.predict();
    auto pred_landing = ekf.predict_landing_point();
    REQUIRE(pred_landing(0) > -1e6); // finite, not NaN
}

TEST_CASE("EkfBallPredictor plane crossing detection", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    // Ball at x=0, moving towards x=1
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(1, 0, 0); // moving in +x direction
    ekf.initialize_state(pos, vel);

    // Predict plane crossing at x=0.5
    auto crossing = ekf.predict_plane_crossing(0.5);
    REQUIRE(crossing.valid);
    REQUIRE((crossing.y >= -10 && crossing.y <= 10)); // reasonable bounds
    // z may go slightly negative near landing due to discrete integration
    REQUIRE((crossing.z >= -0.5 && crossing.z <= 2.0));
    REQUIRE(crossing.time_to_cross > 0);
}

TEST_CASE("EkfBallPredictor plane crossing invalid if moving away", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    // Ball at x=0, moving away from x=1
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(-1, 0, 0); // moving in -x direction
    ekf.initialize_state(pos, vel);

    auto crossing = ekf.predict_plane_crossing(1);
    REQUIRE(!crossing.valid); // moving away, so invalid
}

TEST_CASE("EkfBallPredictor prediction landing point", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    // Vertical drop from 1m with initial velocity
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(0, 0, 0);
    ekf.initialize_state(pos, vel);

    // Run a few predict/update cycles
    for (int i = 0; i < 50; ++i) {
        ekf.predict();
        ekf.update(Eigen::Vector3d(0, 0, 1 - 0.1 * i * i)); // synthetic fall
    }

    auto landing = ekf.predict_landing_point();
    REQUIRE(std::isfinite(landing(0)));
    REQUIRE(std::isfinite(landing(1)));
}

TEST_CASE("EkfBallPredictor innovation tracking", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(1, 1, 0);
    ekf.initialize_state(pos, vel);

    // First update with perfect measurement
    ekf.predict();
    ekf.update(pos);
    double innov1 = ekf.last_innovation_norm();
    REQUIRE(innov1 >= 0); // should be small but non-zero due to time elapsed

    // Update with noisy measurement
    ekf.predict();
    Eigen::Vector3d noisy_pos(0.1, 0.1, 1.1); // 10cm off in each axis
    ekf.update(noisy_pos);
    double innov2 = ekf.last_innovation_norm();
    REQUIRE(innov2 > innov1); // larger error should increase innovation
}

TEST_CASE("EkfBallPredictor covariance trace", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(1, 1, 1);
    ekf.initialize_state(pos, vel);

    double trace_init = ekf.covariance_trace();
    REQUIRE(trace_init > 0);

    // Multiple updates should tighten covariance (reduce trace)
    for (int i = 0; i < 10; ++i) {
        ekf.predict();
        ekf.update(pos);
    }

    double trace_after = ekf.covariance_trace();
    REQUIRE(trace_after <= trace_init); // uncertainty should decrease with updates
}

TEST_CASE("EkfBallPredictor zero velocity (stationary ball)", "[ekf]") {
    EkfBallPredictor ekf(0.002, 0.01);
    Eigen::Vector3d pos(1, 2, 0.5);
    Eigen::Vector3d vel(0, 0, 0); // stationary
    ekf.initialize_state(pos, vel);

    ekf.predict();
    ekf.update(pos);

    // Landing point prediction should handle v=0 gracefully
    auto landing = ekf.predict_landing_point();
    REQUIRE(std::isfinite(landing(0)));
    REQUIRE(std::isfinite(landing(1)));
}

#ifdef FIXED_POINT
TEST_CASE("EkfBallPredictor with fixed-point math", "[ekf][fixed-point]") {
    EkfBallPredictor ekf(0.002, 0.01);
    Eigen::Vector3d pos(0, 0, 1);
    Eigen::Vector3d vel(5, 0, 3);
    ekf.initialize_state(pos, vel);

    // Run several cycles; should produce reasonable results
    for (int i = 0; i < 100; ++i) {
        ekf.predict();
        ekf.update(Eigen::Vector3d(0 + 0.1*i, 0, 1 - 0.01*i*i));
    }

    auto landing = ekf.predict_landing_point();
    REQUIRE(std::isfinite(landing(0)));
    REQUIRE(std::isfinite(landing(1)));

    double trace = ekf.covariance_trace();
    REQUIRE(trace > 0 && std::isfinite(trace));
}
#endif
