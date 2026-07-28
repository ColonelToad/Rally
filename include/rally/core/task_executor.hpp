#pragma once
#include <stdint.h>
#include <iostream>

namespace rally {
namespace core {

// Pins current thread to a specific core (useful for P-core pinning on Linux)
bool configure_realtime_thread(int priority = 80, int core_id = -1);

class ITask {
public:
    virtual ~ITask() = default;
    virtual void execute(uint64_t current_time_us) = 0;
    virtual const char* get_name() const = 0;
};

struct JitterStats {
    int64_t min_jitter_us = 999999;
    int64_t max_jitter_us = -999999;
    uint64_t total_samples = 0;
    
    // Bins: [<10us, 10-50us, 50-100us, 100-500us, >500us]
    uint64_t bins[5] = {0, 0, 0, 0, 0};

    void record(int64_t jitter_us) {
        total_samples++;
        if (jitter_us < min_jitter_us) min_jitter_us = jitter_us;
        if (jitter_us > max_jitter_us) max_jitter_us = jitter_us;

        uint64_t abs_j = jitter_us < 0 ? -jitter_us : jitter_us;
        if (abs_j < 10)       bins[0]++;
        else if (abs_j < 50)  bins[1]++;
        else if (abs_j < 100) bins[2]++;
        else if (abs_j < 500) bins[3]++;
        else                  bins[4]++;
    }

    void print_histogram(const char* task_name) const {
        std::cout << "\n==================================================\n";
        std::cout << " JITTER HISTOGRAM: " << task_name << "\n";
        std::cout << " Total Samples: " << total_samples 
                  << " | Min: " << min_jitter_us << "us | Max: " << max_jitter_us << "us\n";
        std::cout << "--------------------------------------------------\n";
        std::cout << "  < 10 us   : " << bins[0] << "\n";
        std::cout << " 10-50 us   : " << bins[1] << "\n";
        std::cout << " 50-100 us  : " << bins[2] << "\n";
        std::cout << " 100-500 us : " << bins[3] << "\n";
        std::cout << "  > 500 us  : " << bins[4] << "  <-- [Spikes]\n";
        std::cout << "==================================================\n";
    }
};

struct TaskConfig {
    ITask* task;
    uint64_t period_us;
    uint64_t max_runtime_us;
    uint64_t last_run_us;
    JitterStats stats;
};

class TaskExecutor {
public:
    static constexpr int MAX_TASKS = 4;

    TaskExecutor();

    bool register_task(ITask* task, uint32_t frequency_hz, uint32_t deadline_us);

    void run();
    void step();

    const JitterStats& get_task_stats(int task_index) const { return tasks_[task_index].stats; }

private:
    TaskConfig tasks_[MAX_TASKS];
    int task_count_;

    void handle_deadline_miss(const TaskConfig& config, uint64_t actual_runtime_us);
    void log_jitter(TaskConfig& config, int64_t jitter_us);
};

} // namespace core
} // namespace rally