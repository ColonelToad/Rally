#pragma once
#include <stdint.h>

extern "C" {
struct JointState {
    uint64_t timestamp_us;
    double positions[7];
    double velocities[7];
};
static_assert(sizeof(JointState) == 120, "JointState layout changed — update all serializers");
}