#pragma once

#include <vector>
#include <atomic>
#include <cstddef>
#include <utility>

namespace rally {
namespace core {

template <typename T, size_t Capacity>
class SpscRingBuffer {
public:
    SpscRingBuffer() : head_(0), tail_(0) {
        // Enforce capacity to be a power of two for fast bitwise wrapping
        static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of two");
        buffer_.resize(Capacity);
    }

    // Delete copy/move semantics to prevent accidental sharing bugs
    SpscRingBuffer(const SpscRingBuffer&) = delete;
    SpscRingBuffer& operator=(const SpscRingBuffer&) = delete;

    /**
     * @brief Pushes an item into the buffer. 
     * @note MUST be called exclusively by the Producer thread.
     */
    bool push(const T& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t current_head = head_.load(std::memory_order_acquire);

        if (is_full(current_head, current_tail)) {
            return false; // Buffer is full
        }

        buffer_[current_tail & mask()] = item;
        tail_.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    bool push(T&& item) {
        const size_t current_tail = tail_.load(std::memory_order_relaxed);
        const size_t current_head = head_.load(std::memory_order_acquire);

        if (is_full(current_head, current_tail)) {
            return false;
        }

        buffer_[current_tail & mask()] = std::move(item);
        tail_.store(current_tail + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pops an item out of the buffer.
     * @note MUST be called exclusively by the Consumer thread.
     */
    bool pop(T& item) {
        const size_t current_head = head_.load(std::memory_order_relaxed);
        const size_t current_tail = tail_.load(std::memory_order_acquire);

        if (is_empty(current_head, current_tail)) {
            return false; // Buffer is empty
        }

        item = std::move(buffer_[current_head & mask()]);
        head_.store(current_head + 1, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    size_t size() const {
        size_t head = head_.load(std::memory_order_acquire);
        size_t tail = tail_.load(std::memory_order_acquire);
        return (tail >= head) ? (tail - head) : (Capacity - (head - tail));
    }

private:
    constexpr size_t mask() const {
        return Capacity - 1;
    }

    bool is_empty(size_t head, size_t tail) const {
        return head == tail;
    }

    bool is_full(size_t head, size_t tail) const {
        return (tail - head) >= Capacity;
    }

    std::vector<T> buffer_;

    // Align indices to 64-byte cache lines to prevent False Sharing 
    // between the producer and consumer CPU cores.
    alignas(64) std::atomic<size_t> head_;
    alignas(64) std::atomic<size_t> tail_;
};

} // namespace core
} // namespace rally