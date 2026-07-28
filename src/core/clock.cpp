#include "rally/core/clock.hpp"
#include <chrono>
#include <stdexcept>

namespace rally {
namespace core {

// Initialize static members
ClockMode Clock::current_mode_ = ClockMode::REALTIME; 
uint64_t Clock::sim_time_us_ = 0;

void Clock::init(ClockMode mode) {
    current_mode_ = mode;
    sim_time_us_ = 0;
}

uint64_t Clock::now_us() {
    if (current_mode_ == ClockMode::SIMULATED) {
        return sim_time_us_;
    } else {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration_cast<std::chrono::microseconds>(
            now.time_since_epoch()).count();
    }
}

void Clock::update_sim_time(uint64_t sim_time_us) {
    if (current_mode_ == ClockMode::REALTIME) {
        throw std::logic_error("Cannot manually update sim time while in REALTIME mode.");
    }
    // Prevent time from moving backwards
    if (sim_time_us < sim_time_us_) {
        throw std::logic_error("Simulated time cannot flow backwards.");
    }
    sim_time_us_ = sim_time_us;
}

ClockMode Clock::get_mode() {
    return current_mode_;
}

} // namespace core
} // namespace rally