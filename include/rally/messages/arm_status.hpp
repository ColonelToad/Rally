#pragma once
#include <stdint.h>

extern "C" {
struct ArmStatus {
    uint64_t timestamp_us;
    uint8_t arm_id;
    uint8_t current_state; // e.g., 0=idle, 1=moving, 2=error
    uint8_t _padding[6];   // Explicit padding
    double current_end_effector_pos[3];
};
static_assert(sizeof(ArmStatus) == 40, "ArmStatus layout changed — update all serializers");
}