#include "rally/core/task_executor.hpp"
#include "rally/core/clock.hpp"
#include "rally/core/spsc_ring_buffer.hpp"
#include <mujoco/mujoco.h>
#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>

using namespace rally::core;

// Struct to pass telemetry data safely across threads
struct TelemetryMsg {
    double time;
    double angle;
    double torque;
};

// Global SPSC ring buffer with a power-of-two capacity (e.g., 1024 slots)
SpscRingBuffer<TelemetryMsg, 1024> telemetry_queue;

class PhysicsControlTask : public ITask {
public:
    PhysicsControlTask(mjModel* m, mjData* d) : m_(m), d_(d) {}

    void execute(uint64_t /*current_time_us*/) override {
        double current_angle = d_->qpos[0];
        double current_vel   = d_->qvel[0];

        double target_angle = 1.0;
        double kp = 50.0;
        double kd = 5.0;

        double error = target_angle - current_angle;
        double torque = (kp * error) - (kd * current_vel);

        d_->ctrl[0] = torque;
        mj_step(m_, d_);

        // Push telemetry non-blocking into the lock-free ring buffer
        TelemetryMsg msg{d_->time, current_angle, d_->ctrl[0]};
        telemetry_queue.push(msg); // If full, drops frame safely without blocking
    }

    const char* get_name() const override { return "MuJoCo_Sim_500Hz"; }

private:
    mjModel* m_;
    mjData* d_;
};

int main(int, char**) {
    Clock::init(ClockMode::REALTIME);

    mjModel* m = mj_loadXML("assets/pendulum.xml", nullptr, nullptr, 0);
    if (!m) return 1;
    mjData* d = mj_makeData(m);

    // Background consumer thread for printing telemetry (Runs on standard OS scheduler)
    bool logger_running = true;
    std::thread logger_thread([&logger_running]() {
        TelemetryMsg msg;
        while (logger_running) {
            bool received = false;
            // Drain the queue as messages arrive
            while (telemetry_queue.pop(msg)) {
                received = true;
                std::cout << std::fixed << std::setprecision(3)
                          << "[Async Telemetry] Time: " << msg.time << "s | "
                          << "Angle: " << msg.angle << " rad | "
                          << "Torque: " << msg.torque << " Nm\n";
            }
            if (!received) {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        }
    });

    configure_realtime_thread(80, 1);
    
    TaskExecutor executor;
    PhysicsControlTask control_task(m, d);
    executor.register_task(&control_task, 500, 2000);

    std::cout << "[Bridge] Running real-time simulation with lock-free SPSC queue...\n";
    
    while (d->time < 3.0) {
        executor.step();
    }

    logger_running = false;
    if (logger_thread.joinable()) {
        logger_thread.join();
    }

    executor.get_task_stats(0).print_histogram(control_task.get_name());

    mj_deleteData(d);
    mj_deleteModel(m);
    return 0;
}