#ifndef LOCK_FREE_STATE_BUFFER_HPP
#define LOCK_FREE_STATE_BUFFER_HPP

#include <array>
#include <atomic>

// The data payload we want to share
struct BallState {
    std::array<double, 3> pos = {0.0, 0.0, 0.0};
    std::array<double, 3> vel = {0.0, 0.0, 0.0};
    double timestamp = 0.0;
    bool valid = false;
};

// A Single-Producer, Multi-Consumer lock-free ring buffer
template <size_t N = 16>
class LockFreeStateBuffer {
private:
    std::array<BallState, N> buffer;
    std::atomic<size_t> head{0}; 

public:
    /**
     * @brief Pushes a new state to the buffer.
     * @note Called ONLY by the Sensor/HAL thread (Single Producer).
     */
    void push(const BallState& state) {
        size_t current_head = head.load(std::memory_order_relaxed);
        size_t next_head = (current_head + 1) % N;
        
        // Write the data payload first
        buffer[next_head] = state;
        
        // Publish the new head index using release semantics.
        // This guarantees the payload write finishes BEFORE the index updates.
        head.store(next_head, std::memory_order_release);
    }

    /**
     * @brief Gets the most recent ball state.
     * @note Called by Control Threads (Multiple Consumers).
     */
    BallState getLatest() const {
        // Read the head index using acquire semantics.
        // This guarantees we don't read the payload until the index is fully updated.
        size_t current_head = head.load(std::memory_order_acquire);
        
        return buffer[current_head];
    }
};

#endif // LOCK_FREE_STATE_BUFFER_HPP