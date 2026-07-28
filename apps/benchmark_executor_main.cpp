#include "rally/core/task_executor.hpp"
#include "rally/core/clock.hpp"
#include <Eigen/Dense>
#include <iostream>
#include <chrono>

using namespace rally::core;

// Custom 6-DOF typedefs for robot kinematics/dynamics
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class RealisticControlTask : public ITask {
public:
    void execute(uint64_t /*current_time_us*/) override {
        // Simulate a typical 500Hz 6-DOF controller workload
        Matrix6d A = Matrix6d::Random();
        Vector6d b = Vector6d::Random();
        volatile Vector6d x = A.ldlt().solve(b);
        (void)x;
    }
    const char* get_name() const override { return "EigenControl_500Hz"; }
};

int main(int argc, char** argv) {
    Clock::init(ClockMode::REALTIME);

    bool use_rt = false;
    int target_core = 0;

    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--rt") {
            use_rt = true;
        }
        if (std::string(argv[i]) == "--core" && i + 1 < argc) {
            target_core = std::stoi(argv[++i]);
        }
    }

    if (use_rt) {
        // Priority 80, pinned to target P-Core
        configure_realtime_thread(80, target_core);
    } else {
        std::cout << "[Benchmark] Running with default OS scheduler (SCHED_OTHER)\n";
    }

    TaskExecutor executor;
    RealisticControlTask control_task;

    // 500Hz task (2000us period, 500us deadline)
    executor.register_task(&control_task, 500, 500);

    std::cout << "[Benchmark] Running 500Hz control loop for 5 seconds...\n";
    auto start_time = std::chrono::steady_clock::now();

    while (std::chrono::duration_cast<std::chrono::seconds>(
               std::chrono::steady_clock::now() - start_time).count() < 5) {
        executor.step();
    }

    executor.get_task_stats(0).print_histogram(control_task.get_name());

    return 0;
}