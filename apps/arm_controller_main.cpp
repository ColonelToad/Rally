#include "rally/core/task_executor.hpp"
#include "rally/core/clock.hpp"
#include "rally/ipc/zmq_context.hpp"
#include <iostream>

using namespace rally::core;

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

int main() {
    Clock::init(ClockMode::REALTIME);

    TaskExecutor executor;
    ControlTask500Hz control_task;
    SensorFusion100Hz fusion_task;
    Diagnostics10Hz diag_task;

    // Register 500Hz (2000us period, 500us deadline)
    executor.register_task(&control_task, 500, 500);
    // Register 100Hz (10000us period, 2000us deadline)
    executor.register_task(&fusion_task, 100, 2000);
    // Register 10Hz (100000us period, 10000us deadline)
    executor.register_task(&diag_task, 10, 10000);

    executor.run();
    return 0;
}