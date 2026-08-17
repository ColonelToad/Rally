#include "rally/core/task_executor.hpp"
#include "rally/core/clock.hpp"
#include "rally/core/logger.hpp"
#include "rally/ipc/zmq_context.hpp"
#include "rally/messages/play_style_params.hpp"
#include <iostream>
#include <string>

using namespace rally::core;
using namespace rally::ipc;

class ControlTask500Hz : public ITask {
public:
    void execute(uint64_t /*current_time_us*/) override {
        // High-frequency control logic (PID / torque computation)
    }
    const char* get_name() const override { return "Control_500Hz"; }
};

class SensorFusion100Hz : public ITask {
public:
    void execute(uint64_t /*current_time_us*/) override {
        // Medium-frequency state estimation / filtering
    }
    const char* get_name() const override { return "SensorFusion_100Hz"; }
};

class Diagnostics10Hz : public ITask {
public:
    void execute(uint64_t /*current_time_us*/) override {
        std::cout << "[Arm Controller] 10Hz Diagnostics tick...\n";
    }
    const char* get_name() const override { return "Diagnostics_10Hz"; }
};

// Drains coordination_bus's PlayStyleParams updates at a low heartbeat rate.
// The LLC never computes strategy itself (PHILOSOPHY.md Principle 8) — it
// only ever applies whatever the HLC (coordination_bus's RLSEstimator) last
// published. ControlTask500Hz reads current_params() to bias its targeting.
class PlayStyleSync10Hz : public ITask {
public:
    explicit PlayStyleSync10Hz(ZmqSocket& params_pull) : params_pull_(params_pull) {}

    void execute(uint64_t /*current_time_us*/) override {
        // Drain to the latest — an intermediate update between ticks is
        // superseded, not queued, matching the drop-queue pattern brain.py
        // already uses for the LLM's post-rally strategy pushes.
        while (params_pull_.receive(latest_, ZMQ_DONTWAIT)) {
        }
    }
    const char* get_name() const override { return "PlayStyleSync_10Hz"; }

    const PlayStyleParams& current_params() const { return latest_; }

private:
    ZmqSocket& params_pull_;
    PlayStyleParams latest_{};
};

int main(int argc, char** argv) {
    Clock::init(ClockMode::REALTIME);

    // Pin this thread's busy-spin loop away from core 0, where the OS,
    // WSLg compositor, and the Python viewer's render thread tend to land.
    int target_core = 1;
    std::string side = "left";
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--core" && i + 1 < argc) {
            target_core = std::stoi(argv[++i]);
        }
        if (std::string(argv[i]) == "--side" && i + 1 < argc) {
            side = argv[++i];
        }
    }
    configure_realtime_thread(80, target_core);

    // One telemetry file per arm process, so left/right jitter logs don't
    // collide when both are run concurrently.
    Logger::instance().start("rally_telemetry_" + side + ".log");

    ZmqContext zmq_ctx;
    const std::string params_url = (side == "left")
        ? "ipc:///tmp/rally/play_style_left.sock"
        : "ipc:///tmp/rally/play_style_right.sock";
    ZmqSocket params_pull(zmq_ctx, SocketType::PULL);
    params_pull.connect(params_url);

    TaskExecutor executor;
    ControlTask500Hz control_task;
    SensorFusion100Hz fusion_task;
    Diagnostics10Hz diag_task;
    PlayStyleSync10Hz play_style_task(params_pull);

    // Register 500Hz (2000us period, 500us deadline)
    executor.register_task(&control_task, 500, 500);
    // Register 100Hz (10000us period, 2000us deadline)
    executor.register_task(&fusion_task, 100, 2000);
    // Register 10Hz (100000us period, 10000us deadline)
    executor.register_task(&diag_task, 10, 10000);
    // Register 10Hz (100000us period, 10000us deadline) — same rate as
    // diagnostics; this is a heartbeat sync, not a real-time control path.
    executor.register_task(&play_style_task, 10, 10000);

    executor.run();
    return 0;
}