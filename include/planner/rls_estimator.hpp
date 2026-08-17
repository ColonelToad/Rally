#ifndef RLS_ESTIMATOR_HPP
#define RLS_ESTIMATOR_HPP

#include "rally/core/task_executor.hpp"
#include "rally/messages/play_style_params.hpp"
#include "planner/analytical_ik.hpp"
#include <Eigen/Dense>
#include <atomic>
#include <array>
#include <algorithm>

// PlayStyleParams::arm_id convention: 0 = left arm, 1 = right arm.
enum class ArmSide : uint8_t { LEFT = 0, RIGHT = 1 };

// The subset of rally::messages::RallyOutcome (the wire message) relevant
// to one arm's estimator, already resolved to "did MY arm lose the point."
// Named distinctly from the wire struct to avoid a same-name collision when
// both headers are included together.
struct RallyOutcomeSample {
    double last_ball_y;
    double rally_length;
    bool arm_lost_point;
};

// Recursive Least Squares estimator for one arm's return-placement bias.
// theta = [target_offset_y, aggression_factor, reaction_margin]
//
// Registered with TaskExecutor at a low heartbeat frequency purely for
// housekeeping (execute()) — the actual learning update fires directly
// from on_rally_outcome(), called from the ZMQ handler that receives the
// post-rally summary. This mirrors how brain.py's evaluate_rally_async is
// invoked directly from viewer.py's keyboard callback, not on a timer.
class RLSEstimator : public rally::core::ITask {
public:
    explicit RLSEstimator(ArmSide side,
                           const std::array<double, 3>& mount_world_pos,
                           double lambda = 0.97)
        : side_(side), mount_world_pos_(mount_world_pos), lambda_(lambda), version_(0) {
        theta_ = Eigen::Vector3d(0.0, 1.0, 0.0); // neutral aim, unit aggression, no margin bias
        P_ = Eigen::Matrix3d::Identity() * 10.0; // high initial uncertainty
        snapshot_ = PlayStyleParams{0, static_cast<uint8_t>(side_), {0, 0, 0, 0, 0, 0, 0}, 0.0, 1.0, 0.0};
    }

    // Called directly from the ZMQ handler when a rally ends — NOT driven
    // by TaskExecutor's frequency-based scheduling (see class comment).
    void on_rally_outcome(const RallyOutcomeSample& outcome, uint64_t timestamp_us) {
        if (!outcome.arm_lost_point) {
            // A win means the current bias was sufficient; skip the update.
            // Injecting a self-referential target here would falsely shrink
            // P without adding information, and fight the forgetting factor.
            return;
        }

        Eigen::Vector3d x = build_feature_vector(outcome);
        double y_target = compute_target_signal(outcome);

        // Standard RLS update:
        // K = P*x / (lambda + x^T*P*x)
        // e = y_target - x^T*theta
        // theta = theta + K*e
        // P = (P - K*x^T*P) / lambda
        Eigen::Vector3d Px = P_ * x;
        double denom = lambda_ + x.dot(Px);
        Eigen::Vector3d K = Px / denom;

        double error = y_target - x.dot(theta_);
        theta_ += K * error;
        P_ = (P_ - K * Px.transpose()) / lambda_;

        publish_snapshot(timestamp_us);
    }

    // Lock-free read path for the 500Hz trajectory planning loop.
    // Seqlock: readers never block the (infrequent) writer, and never
    // block each other, at the cost of an occasional retry on tear.
    PlayStyleParams get_current_params() const {
        PlayStyleParams out;
        uint64_t v_before, v_after;
        do {
            v_before = version_.load(std::memory_order_acquire);
            out = snapshot_;
            v_after = version_.load(std::memory_order_acquire);
        } while (v_before != v_after || (v_before & 1));
        return out;
    }

    // Housekeeping only (e.g. staleness checks) — the real learning
    // trigger is on_rally_outcome(), not this periodic tick.
    void execute(uint64_t /*current_time_us*/) override {}
    const char* get_name() const override {
        return side_ == ArmSide::LEFT ? "RLSEstimator_Left" : "RLSEstimator_Right";
    }

private:
    Eigen::Vector3d build_feature_vector(const RallyOutcomeSample& outcome) const {
        // [ball_y_at_miss, normalized rally_length, bias]
        const double kMaxExpectedRally = 20.0;
        double normalized_length = std::min(outcome.rally_length / kMaxExpectedRally, 1.0);
        return Eigen::Vector3d(outcome.last_ball_y, normalized_length, 1.0);
    }

    double compute_target_signal(const RallyOutcomeSample& outcome) const {
        // On a loss, aim toward where the opponent just beat us.
        return outcome.last_ball_y;
    }

    // Validates the proposed offset against the arm's real reachability
    // (joint limits included), reusing AnalyticalIK::computeIK as the
    // single source of truth rather than re-deriving a workspace bound.
    bool is_reachable(double target_offset_y) const {
        double local_x = 0.3; // approximate nominal hit distance in front of the mount
        double local_y = target_offset_y - mount_world_pos_[1];
        double local_z = mount_world_pos_[2];
        std::array<double, 7> q_scratch{};
        return AnalyticalIK::computeIK(local_x, local_y, local_z, 0.0, q_scratch);
    }

    void publish_snapshot(uint64_t timestamp_us) {
        double proposed_offset = theta_(0);
        if (!is_reachable(proposed_offset)) {
            // Clamp by holding the last known-reachable offset rather than
            // publishing a target the IK solver cannot service.
            proposed_offset = snapshot_.target_offset_y;
            theta_(0) = proposed_offset;
        }

        uint64_t v = version_.load(std::memory_order_relaxed);
        version_.store(v + 1, std::memory_order_release); // odd = write in progress

        snapshot_.timestamp_us = timestamp_us;
        snapshot_.arm_id = static_cast<uint8_t>(side_);
        snapshot_.target_offset_y = proposed_offset;
        snapshot_.aggression_factor = theta_(1);
        snapshot_.reaction_margin = theta_(2);

        version_.store(v + 2, std::memory_order_release); // even = stable
    }

    ArmSide side_;
    std::array<double, 3> mount_world_pos_;
    double lambda_;

    Eigen::Vector3d theta_;
    Eigen::Matrix3d P_;

    PlayStyleParams snapshot_;
    std::atomic<uint64_t> version_;
};

#endif // RLS_ESTIMATOR_HPP
