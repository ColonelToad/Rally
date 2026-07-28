#pragma once
#include <stdint.h>

extern "C" {
struct ArmAssignment {
    uint64_t timestamp_us;
    uint8_t assigned_arm; // 0 = Arm A, 1 = Arm B, 2 = Neither
    uint8_t _padding[7];  // Explicit padding to 8-byte boundary
    uint64_t intercept_time_us;
    double intercept_point[3];
};
static_assert(sizeof(ArmAssignment) == 48, "ArmAssignment layout changed — update all serializers");
}