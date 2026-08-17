#pragma once
#include <stdint.h>

extern "C" {
struct PlayStyleParams {
    uint64_t timestamp_us;
    uint8_t arm_id;
    uint8_t _padding[7];
    double target_offset_y;
    double aggression_factor;
    double reaction_margin;
};
static_assert(sizeof(PlayStyleParams) == 40, "PlayStyleParams layout changed — update all serializers");
}
