#include "rally/core/task_executor.hpp"
#include "rally/core/clock.hpp" // Fixes 'Clock' and 'ClockMode' errors
#include "rally/core/logger.hpp"
#include <sys/mman.h>           // For mlockall
#include <pthread.h>
#include <sched.h>
#include <iostream>
#include <thread>               // Fixes 'std::this_thread' error
#include <chrono>               // Fixes 'std::chrono' error
#include <cstring>

namespace rally {
namespace core {

bool pin_thread_to_core(int core_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);
    
    pthread_t current_thread = pthread_self();
    int rc = pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
    return (rc == 0);
}

bool configure_realtime_thread(int priority, int core_id) {
    // 1. Lock process memory into RAM (prevents page-fault latency spikes)
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[RealTime] WARNING: mlockall() failed! Memory may swap. "
                  << "(Requires root or CAP_IPC_LOCK)\n";
    } else {
        std::cout << "[RealTime] Memory locked into RAM (mlockall success).\n";
    }

    // 2. Pin thread to specific CPU core if requested
    if (core_id >= 0) {
        if (!pin_thread_to_core(core_id)) {
            std::cerr << "[RealTime] FAILED to pin thread to CPU Core " << core_id << "\n";
        } else {
            std::cout << "[RealTime] Pinned thread to CPU Core " << core_id << "\n";
        }
    }

    // 3. Set POSIX SCHED_FIFO scheduling policy and priority
    pthread_t current_thread = pthread_self();
    struct sched_param param;
    param.sched_priority = priority;

    int rc = pthread_setschedparam(current_thread, SCHED_FIFO, &param);
    if (rc != 0) {
        std::cerr << "[RealTime] FAILED to set SCHED_FIFO (error code: " << rc << "). "
                  << "Did you run with 'sudo' or configure /etc/security/limits.conf?\n";
        return false;
    }

    std::cout << "[RealTime] SCHED_FIFO policy enabled at priority " << priority << "\n";
    return true;
}

TaskExecutor::TaskExecutor() : task_count_(0) {
    for (int i = 0; i < MAX_TASKS; ++i) {
        tasks_[i] = {nullptr, 0, 0, 0, {}};
    }
    log_producer_slot_ = Logger::instance().register_producer();
}

bool TaskExecutor::register_task(ITask* task, uint32_t frequency_hz, uint32_t deadline_us) {
    if (task_count_ >= MAX_TASKS || frequency_hz == 0) {
        return false;
    }

    TaskConfig& config = tasks_[task_count_];
    config.task = task;
    config.period_us = 1000000 / frequency_hz;
    config.max_runtime_us = deadline_us;
    config.last_run_us = Clock::now_us();
    config.stats = {};

    task_count_++;
    return true;
}

void TaskExecutor::run() {
    std::cout << "[TaskExecutor] Entering real-time scheduling loop...\n";
    while (true) {
        step();
    }
}

void TaskExecutor::step() {
    uint64_t current_time = Clock::now_us();
    bool idle = true;
    
    // Track when the absolute closest next task is due
    uint64_t next_wake_us = UINT64_MAX; 

    for (int i = 0; i < task_count_; ++i) {
        TaskConfig& config = tasks_[i];
        uint64_t due_time = config.last_run_us + config.period_us;

        if (current_time >= due_time) {
            idle = false;

            uint64_t exec_start = Clock::now_us();
            config.task->execute(current_time);
            uint64_t exec_end = Clock::now_us();

            uint64_t actual_runtime = exec_end - exec_start;
            int64_t jitter = static_cast<int64_t>(exec_start) - static_cast<int64_t>(due_time);

            bool deadline_missed = actual_runtime > config.max_runtime_us;
            log_jitter(config, jitter, actual_runtime, deadline_missed);

            if (deadline_missed) {
                handle_deadline_miss(config, actual_runtime);
            }

            // Advance period to prevent phase drift
            config.last_run_us += config.period_us;

            // Anti-windup
            if (Clock::now_us() > config.last_run_us + (config.period_us * 2)) {
                config.last_run_us = Clock::now_us();
            }
        } else if (due_time < next_wake_us) {
            next_wake_us = due_time;
        }
    }

    if (idle && Clock::get_mode() == ClockMode::REALTIME) {
        // Pure busy-spin: Don't yield to the OS at all!
        // (Will use 100% of a CPU core, but guarantees lowest possible jitter)
        asm volatile("pause" ::: "memory"); // Yields CPU pipeline, but not the thread
    }
}

void TaskExecutor::handle_deadline_miss(const TaskConfig& config, uint64_t actual_runtime_us) {
    std::cerr << "[WARNING] Task '" << config.task->get_name()
              << "' missed deadline! Allowed: " << config.max_runtime_us
              << "us, Took: " << actual_runtime_us << "us. Skipping remainder and recovering.\n";
}

void TaskExecutor::log_jitter(TaskConfig& config, int64_t jitter_us, uint64_t runtime_us, bool deadline_missed) {
    config.stats.record(jitter_us);

    LogRecord record;
    record.timestamp_us = Clock::now_us();
    record.type = LogRecordType::TASK_JITTER;
    std::strncpy(record.task_jitter.task_name, config.task->get_name(), sizeof(record.task_jitter.task_name) - 1);
    record.task_jitter.task_name[sizeof(record.task_jitter.task_name) - 1] = '\0';
    record.task_jitter.jitter_us = jitter_us;
    record.task_jitter.runtime_us = runtime_us;
    record.task_jitter.deadline_missed = deadline_missed ? 1 : 0;
    Logger::instance().log(log_producer_slot_, record);
}

} // namespace core
} // namespace rally