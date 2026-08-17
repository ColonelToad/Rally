#include "rally/ipc/zmq_context.hpp"
#include "rally/messages/arm_status.hpp"
#include "rally/messages/rally_outcome.hpp"
#include "rally/messages/play_style_params.hpp"
#include "planner/rls_estimator.hpp"
#include <iostream>
#include <chrono>
#include <thread>

using namespace rally::ipc;

const std::string ARM_A_STATUS_PULL_URL = "ipc:///tmp/rally/arm_a_status.sock";
// RallyOutcome arrives from mujoco_bridge over TCP (crosses the physics/HLC
// process boundary), same transport family as the other mujoco_bridge sockets.
const std::string RALLY_OUTCOME_PULL_URL = "tcp://localhost:5559";
const std::string PLAY_STYLE_LEFT_PUSH_URL = "ipc:///tmp/rally/play_style_left.sock";
const std::string PLAY_STYLE_RIGHT_PUSH_URL = "ipc:///tmp/rally/play_style_right.sock";
const double LOOP_RATE_HZ = 100.0;
const double CYCLE_TIME_MS = 1000.0 / LOOP_RATE_HZ;

int main() {
    std::cout << "[Coordination Bus] Starting up..." << std::endl;
    ZmqContext ctx;

    // Bind PULL socket to receive statuses from Arm A
    ZmqSocket status_pull(ctx, SocketType::PULL);
    status_pull.bind(ARM_A_STATUS_PULL_URL);

    // Connect (not bind) — mujoco_bridge owns and binds this endpoint.
    ZmqSocket outcome_pull(ctx, SocketType::PULL);
    outcome_pull.connect(RALLY_OUTCOME_PULL_URL);

    ZmqSocket play_style_left_push(ctx, SocketType::PUSH);
    play_style_left_push.bind(PLAY_STYLE_LEFT_PUSH_URL);
    ZmqSocket play_style_right_push(ctx, SocketType::PUSH);
    play_style_right_push.bind(PLAY_STYLE_RIGHT_PUSH_URL);

    // Mount world positions from panda_hit_scene.xml — asymmetric by
    // construction (left covers left+center, right covers right flank),
    // so each arm gets its own independently-converging estimator.
    RLSEstimator left_estimator(ArmSide::LEFT, {-1.5, 0.3, 0.76});
    RLSEstimator right_estimator(ArmSide::RIGHT, {-1.5, -0.4, 0.76});

    ArmStatus incoming_status{};
    RallyOutcome incoming_outcome{};

    std::cout << "[Coordination Bus] Alive. Polling for arm status at 100Hz...\n";

    while (true) {
        // Non-blocking read (ZMQ_DONTWAIT). The HLC must not get stuck waiting on an LLC.
        while (status_pull.receive(incoming_status, ZMQ_DONTWAIT)) {
            // Drain the queue and process the latest status
        }

        if (incoming_status.timestamp_us > 0) {
            // Just prove we got it
            // In reality, this will update the ownership arbiter state
        }

        // Drain rally outcomes and route each to the arm that lost the point.
        // Only the losing side's estimator updates (see RLSEstimator::on_rally_outcome).
        while (outcome_pull.receive(incoming_outcome, ZMQ_DONTWAIT)) {
            bool left_lost = (incoming_outcome.losing_arm_id == 0);
            RLSEstimator& estimator = left_lost ? left_estimator : right_estimator;

            RallyOutcomeSample sample{
                incoming_outcome.last_ball_y,
                static_cast<double>(incoming_outcome.rally_length),
                /*arm_lost_point=*/true
            };
            estimator.on_rally_outcome(sample, incoming_outcome.timestamp_us);

            PlayStyleParams params_out = estimator.get_current_params();
            if (left_lost) {
                play_style_left_push.send(params_out, ZMQ_DONTWAIT);
            } else {
                play_style_right_push.send(params_out, ZMQ_DONTWAIT);
            }
        }

        // Sleep to enforce 100Hz HLC rate
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(CYCLE_TIME_MS)));
    }

    return 0;
}