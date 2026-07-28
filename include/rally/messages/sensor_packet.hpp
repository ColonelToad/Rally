#pragma once
#include <stdint.h>

extern "C" {
struct SensorPacket {
    uint64_t timestamp_us;
    uint64_t sequence_number;
    double joint_positions[7];
    double joint_velocities[7];
    double joint_torques[7]; 
};
static_assert(sizeof(SensorPacket) == 184, "SensorPacket layout changed — update all serializers");
}