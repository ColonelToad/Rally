#pragma once

#include "rally/core/log_record.hpp"
#include "rally/core/spsc_ring_buffer.hpp"
#include "rally/core/clock.hpp"
#include <atomic>
#include <thread>
#include <fstream>
#include <cstring>
#include <array>
#include <string>

namespace rally {
namespace core {

// One binary timestamped record stream per calling thread. Each producer
// thread gets its own lock-free SpscRingBuffer (matching the existing
// single-producer discipline used elsewhere in this codebase); a single
// background drain thread round-robins across all registered buffers and
// appends to one file, so there is one log on disk, not one per thread.
//
// Fixed memory cost, zero heap allocation on the logging call path itself
// (SpscRingBuffer::push copies into a pre-sized array) — the drain thread
// is the only place doing file I/O, kept off every hot path.
class Logger {
public:
    static constexpr size_t kBufferCapacity = 4096; // power of two, per PHILOSOPHY.md fixed-size discipline
    static constexpr int kMaxProducers = 4;          // left arm, right arm, physics/HAL, spare

    static Logger& instance() {
        static Logger logger;
        return logger;
    }

    // Called once per producer thread before any log() calls from it.
    // Returns the slot index to pass to log() from that thread.
    int register_producer() {
        int slot = producer_count_.fetch_add(1, std::memory_order_relaxed);
        return slot < kMaxProducers ? slot : -1;
    }

    void start(const std::string& path) {
        if (running_.exchange(true)) return; // already started
        out_.open(path, std::ios::binary | std::ios::trunc);
        drain_thread_ = std::thread(&Logger::drain_loop, this);
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (drain_thread_.joinable()) drain_thread_.join();
        out_.close();
    }

    // Zero-allocation: copies a pre-built LogRecord into this producer's
    // ring buffer. Never blocks — a full buffer silently drops the record
    // rather than stall the hot-path caller.
    void log(int producer_slot, const LogRecord& record) {
        if (producer_slot < 0 || producer_slot >= kMaxProducers) return;
        buffers_[producer_slot].push(record);
    }

    ~Logger() { stop(); }

private:
    Logger() : producer_count_(0), running_(false) {}
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    void drain_loop() {
        LogRecord record;
        while (running_.load(std::memory_order_acquire)) {
            bool drained_any = false;
            for (auto& buf : buffers_) {
                while (buf.pop(record)) {
                    out_.write(reinterpret_cast<const char*>(&record), sizeof(LogRecord));
                    drained_any = true;
                }
            }
            if (drained_any) {
                out_.flush();
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        // Final drain after running_ flips false, so records logged right
        // up to shutdown aren't lost.
        for (auto& buf : buffers_) {
            while (buf.pop(record)) {
                out_.write(reinterpret_cast<const char*>(&record), sizeof(LogRecord));
            }
        }
        out_.flush();
    }

    std::array<SpscRingBuffer<LogRecord, kBufferCapacity>, kMaxProducers> buffers_;
    std::atomic<int> producer_count_;
    std::atomic<bool> running_;
    std::thread drain_thread_;
    std::ofstream out_;
};

} // namespace core
} // namespace rally
