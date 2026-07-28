#pragma once
#include <stdint.h>

extern "C" {
struct TorqueCommand {
    uint64_t timestamp_us;
    uint64_t sequence_number;
    double joint_torques[7]; // 7 DOF for Franka Panda
};
static_assert(sizeof(TorqueCommand) == 72, "TorqueCommand layout changed — update all serializers");
}