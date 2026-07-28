#pragma once
#include <stdint.h>

extern "C" {
struct BallState {
    uint64_t timestamp_us;
    double position[3];
    double velocity[3];
    double confidence; 
};
static_assert(sizeof(BallState) == 64, "BallState layout changed — update all serializers");
}