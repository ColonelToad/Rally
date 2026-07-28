#pragma once
#include <stdint.h>

namespace rally {
namespace core {

enum class ClockMode {
    REALTIME,  // Driven by system hardware clock (std::chrono)
    SIMULATED  // Driven by MuJoCo step counter
};

class Clock {
public:
    static void init(ClockMode mode);
    
    // The single source of truth for time across the entire codebase
    static uint64_t now_us();
    
    // Only used by the MuJoCo Bridge in SIMULATED mode
    static void update_sim_time(uint64_t sim_time_us);

    static ClockMode get_mode();

private:
    static ClockMode current_mode_;
    static uint64_t sim_time_us_;
};

} // namespace core
} // namespace rally