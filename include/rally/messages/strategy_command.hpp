#pragma once
#include <stdint.h>

extern "C" {
struct StrategyCommand {
    uint64_t timestamp_us;
    uint64_t mode; // e.g., 0=rally, 1=aim_left, 2=defensive (uint64_t avoids padding)
    double target_zone_x;
    double target_zone_y;
};
static_assert(sizeof(StrategyCommand) == 32, "StrategyCommand layout changed — update all serializers");
}