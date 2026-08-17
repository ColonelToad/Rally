#pragma once
#include <stdint.h>

extern "C" {
struct RallyOutcome {
    uint64_t timestamp_us;
    uint8_t losing_arm_id;  // 0 = left, 1 = right
    uint8_t _padding[7];
    double last_ball_y;
    double last_ball_x;
    uint32_t rally_length;  // count of paddle contacts this rally
    uint8_t _padding2[4];
};
static_assert(sizeof(RallyOutcome) == 40, "RallyOutcome layout changed — update all serializers");
}
