#pragma once
#include <stdint.h>

namespace rally {
namespace core {

// One tagged, fixed-size record format for every subsystem — the live
// writer and the offline replayer/dashboard read the same schema
// (PHILOSOPHY.md Principle 6: "one schema, not separate formats").
enum class LogRecordType : uint8_t {
    TASK_JITTER = 0,
    EKF_FUSION = 1,
    ARBITER_DECISION = 2,
    HAL_ROUNDTRIP = 3,
    RLS_CONVERGENCE = 4,
};

struct TaskJitterPayload {
    char task_name[24]; // fits every current ITask::get_name() in full, e.g. "SensorFusion_100Hz"
    int64_t jitter_us;
    uint64_t runtime_us;
    uint8_t deadline_missed;
    uint8_t _padding[7];
};
static_assert(sizeof(TaskJitterPayload) == 48, "TaskJitterPayload layout changed");

struct EkfFusionPayload {
    double predicted_landing_x;
    double predicted_landing_y;
    double innovation_norm;    // magnitude of measurement residual
    double covariance_trace;
};
static_assert(sizeof(EkfFusionPayload) == 32, "EkfFusionPayload layout changed");

struct ArbiterDecisionPayload {
    uint8_t assigned_owner; // 0=none, 1=left, 2=right
    uint8_t ball_zone;      // 0=left, 1=center, 2=right
    uint8_t _padding[6];
    double confidence;
    double ball_pos_x;
};
static_assert(sizeof(ArbiterDecisionPayload) == 24, "ArbiterDecisionPayload layout changed");

struct HalRoundtripPayload {
    double roundtrip_latency_us;
    uint8_t _padding[24];
};
static_assert(sizeof(HalRoundtripPayload) == 32, "HalRoundtripPayload layout changed");

struct RLSConvergencePayload {
    uint8_t arm_id;           // 0=left, 1=right
    uint8_t _padding[7];
    double target_offset_y;   // theta[0]
    double aggression_factor; // theta[1]
    double reaction_margin;   // theta[2]
};
static_assert(sizeof(RLSConvergencePayload) == 32, "RLSConvergencePayload layout changed");

// Fixed-size union-like record. Every record is the same width on disk
// regardless of type, so the replayer can seek/scan without parsing a
// variable-length format.
struct LogRecord {
    uint64_t timestamp_us;
    LogRecordType type;
    uint8_t _padding[7];
    union {
        TaskJitterPayload task_jitter;
        EkfFusionPayload ekf_fusion;
        ArbiterDecisionPayload arbiter_decision;
        HalRoundtripPayload hal_roundtrip;
        RLSConvergencePayload rls_convergence;
    };

    LogRecord() : timestamp_us(0), type(LogRecordType::TASK_JITTER), _padding{}, task_jitter{} {}
};
static_assert(sizeof(LogRecord) == 64, "LogRecord layout changed — update the offline replayer");

} // namespace core
} // namespace rally
