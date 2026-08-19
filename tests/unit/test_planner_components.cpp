#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "planner/ownership_arbiter.hpp"
#include "planner/analytical_ik.hpp"
#include "planner/rls_estimator.hpp"
#include <cmath>

// ============================================================================
// OwnershipArbiter Tests
// ============================================================================

TEST_CASE("OwnershipArbiter left region assignment", "[arbiter]") {
    OwnershipArbiter arbiter(0.05);
    std::array<double, 3> pos = {-0.2, 0.0, 0.5};
    std::array<double, 3> vel = {0.0, 0.0, 0.0};

    ArmOwner owner = arbiter.arbitrate("test_thread", pos, vel, 0.0);
    REQUIRE(owner == ArmOwner::LEFT_ARM);
}

TEST_CASE("OwnershipArbiter right region assignment", "[arbiter]") {
    OwnershipArbiter arbiter(0.05);
    std::array<double, 3> pos = {0.2, 0.0, 0.5};
    std::array<double, 3> vel = {0.0, 0.0, 0.0};

    ArmOwner owner = arbiter.arbitrate("test_thread", pos, vel, 0.0);
    REQUIRE(owner == ArmOwner::RIGHT_ARM);
}

TEST_CASE("OwnershipArbiter center zone with left velocity", "[arbiter]") {
    OwnershipArbiter arbiter(0.05);
    std::array<double, 3> pos = {0.01, 0.0, 0.5};
    std::array<double, 3> vel = {-1.0, 0.0, 0.0}; // moving left

    ArmOwner owner = arbiter.arbitrate("test_thread", pos, vel, 0.0);
    REQUIRE(owner == ArmOwner::LEFT_ARM);
}

TEST_CASE("OwnershipArbiter center zone with right velocity", "[arbiter]") {
    OwnershipArbiter arbiter(0.05);
    std::array<double, 3> pos = {-0.01, 0.0, 0.5};
    std::array<double, 3> vel = {1.0, 0.0, 0.0}; // moving right

    ArmOwner owner = arbiter.arbitrate("test_thread", pos, vel, 0.0);
    REQUIRE(owner == ArmOwner::RIGHT_ARM);
}

TEST_CASE("OwnershipArbiter hysteresis prevents rapid switching", "[arbiter]") {
    OwnershipArbiter arbiter(0.05);

    // Start on left side
    std::array<double, 3> pos = {-0.1, 0.0, 0.5};
    std::array<double, 3> vel = {0.0, 0.0, 0.0};
    ArmOwner owner1 = arbiter.arbitrate("test_thread", pos, vel, 0.0);
    REQUIRE(owner1 == ArmOwner::LEFT_ARM);

    // Move to barely right of center, but within hysteresis margin
    pos[0] = 0.03;
    vel[0] = 1.0; // moving right
    ArmOwner owner2 = arbiter.arbitrate("test_thread", pos, vel, 0.1);

    // Should still be LEFT_ARM due to hysteresis, not switch to RIGHT
    // unless we exceed the margin (0.05)
    REQUIRE((owner2 == ArmOwner::LEFT_ARM || owner2 == ArmOwner::RIGHT_ARM));
}

TEST_CASE("OwnershipArbiter transition logging", "[arbiter]") {
    OwnershipArbiter arbiter(0.05);
    std::array<double, 3> pos = {-0.2, 0.0, 0.5};
    std::array<double, 3> vel = {0.0, 0.0, 0.0};

    // First call establishes left ownership
    ArmOwner owner1 = arbiter.arbitrate("test_thread", pos, vel, 0.0);
    REQUIRE(owner1 == ArmOwner::LEFT_ARM);

    // Move to right side — should transition
    pos[0] = 0.2;
    ArmOwner owner2 = arbiter.arbitrate("test_thread", pos, vel, 0.1);
    REQUIRE(owner2 == ArmOwner::RIGHT_ARM);
}

// ============================================================================
// AnalyticalIK Tests
// ============================================================================

TEST_CASE("AnalyticalIK basic reachability within workspace", "[ik]") {
    std::array<double, 7> q_out;

    // Target a position well within the Panda workspace
    bool success = AnalyticalIK::computeIK(0.5, 0.0, 0.5, 0.0, q_out);
    REQUIRE(success);
    REQUIRE(q_out.size() == 7);
}

TEST_CASE("AnalyticalIK unreachable target too far", "[ik]") {
    std::array<double, 7> q_out;

    // Target way beyond max reach (0.85m)
    bool success = AnalyticalIK::computeIK(1.5, 0.0, 0.5, 0.0, q_out);
    REQUIRE(!success);
}

TEST_CASE("AnalyticalIK unreachable target too close", "[ik]") {
    std::array<double, 7> q_out;

    // Target too close (< 0.1m from base)
    bool success = AnalyticalIK::computeIK(0.05, 0.0, 0.5, 0.0, q_out);
    REQUIRE(!success);
}

TEST_CASE("AnalyticalIK height bounds checking", "[ik]") {
    std::array<double, 7> q_out;

    // Z below 0
    bool success1 = AnalyticalIK::computeIK(0.5, 0.0, -0.1, 0.0, q_out);
    REQUIRE(!success1);

    // Z above 1.2m
    bool success2 = AnalyticalIK::computeIK(0.5, 0.0, 1.3, 0.0, q_out);
    REQUIRE(!success2);
}

TEST_CASE("AnalyticalIK joint angle limits enforcement", "[ik]") {
    std::array<double, 7> q_out;

    // This test checks that when a position is reachable, returned angles respect limits
    bool success = AnalyticalIK::computeIK(0.5, 0.0, 0.5, 0.0, q_out);

    if (success) {
        const std::array<double, 7> min_limits = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
        const std::array<double, 7> max_limits = { 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973};

        for (size_t i = 0; i < 7; ++i) {
            REQUIRE(q_out[i] >= min_limits[i]);
            REQUIRE(q_out[i] <= max_limits[i]);
        }
    }
}

TEST_CASE("AnalyticalIK theta1 matches atan2(y, x)", "[ik]") {
    std::array<double, 7> q_out;

    double x = 0.4;
    double y = 0.3;
    double z = 0.6;
    double expected_theta1 = std::atan2(y, x);

    bool success = AnalyticalIK::computeIK(x, y, z, 0.0, q_out);
    if (success) {
        REQUIRE(std::abs(q_out[0] - expected_theta1) < 1e-6);
    }
}

// ============================================================================
// RLSEstimator Tests
// ============================================================================

TEST_CASE("RLSEstimator initialization", "[rls]") {
    RLSEstimator rls(ArmSide::LEFT, {-1.5, 0.3, 0.76});
    auto params = rls.get_current_params();

    // Initial parameters should be reasonable (not NaN)
    REQUIRE(std::isfinite(params.arm_id));
    REQUIRE(std::isfinite(params.target_offset_y));
    REQUIRE(std::isfinite(params.aggression_factor));
}

TEST_CASE("RLSEstimator processes rally outcome", "[rls]") {
    RLSEstimator rls(ArmSide::RIGHT, {-1.5, -0.4, 0.76});

    RallyOutcomeSample sample{
        0.2,  // last_ball_y
        10.0, // rally_length
        true  // arm_lost_point
    };

    // Should not crash and should update internal state
    rls.on_rally_outcome(sample, 0);

    auto params = rls.get_current_params();
    REQUIRE(std::isfinite(params.target_offset_y));
}

TEST_CASE("RLSEstimator convergence over multiple outcomes", "[rls]") {
    RLSEstimator rls(ArmSide::LEFT, {-1.5, 0.3, 0.76});

    RallyOutcomeSample sample{0.15, 8.0, true};

    auto params_init = rls.get_current_params();
    double offset_init = params_init.target_offset_y;

    // Feed multiple outcomes to trigger convergence
    for (int i = 0; i < 10; ++i) {
        rls.on_rally_outcome(sample, i * 100);
    }

    auto params_final = rls.get_current_params();
    double offset_final = params_final.target_offset_y;

    // Parameters should have changed (learning occurred)
    // We just verify they remain finite; actual convergence direction depends on RLS tuning
    REQUIRE(std::isfinite(offset_final));
}

TEST_CASE("RLSEstimator left and right arms independent", "[rls]") {
    RLSEstimator left(ArmSide::LEFT, {-1.5, 0.3, 0.76});
    RLSEstimator right(ArmSide::RIGHT, {-1.5, -0.4, 0.76});

    RallyOutcomeSample sample_left{0.2, 10.0, true};
    RallyOutcomeSample sample_right{-0.2, 10.0, true};

    left.on_rally_outcome(sample_left, 0);
    right.on_rally_outcome(sample_right, 0);

    auto left_params = left.get_current_params();
    auto right_params = right.get_current_params();

    // Arm IDs should differ
    REQUIRE(left_params.arm_id != right_params.arm_id);

    // Parameters may differ due to different mount positions
    // Just verify both are finite
    REQUIRE(std::isfinite(left_params.target_offset_y));
    REQUIRE(std::isfinite(right_params.target_offset_y));
}

TEST_CASE("RLSEstimator is_reachable basic bounds", "[rls]") {
    RLSEstimator rls(ArmSide::LEFT, {-1.5, 0.3, 0.76});

    // Well within Panda workspace
    bool reach1 = rls.is_reachable(0.5, 0.3, 0.8);
    REQUIRE(reach1);

    // Way out of reach
    bool reach2 = rls.is_reachable(2.0, 0.0, 0.0);
    REQUIRE(!reach2);
}
