#pragma once

#include "core/types.hpp"

namespace sim {

// MPI message tags for communication multiplexing
enum MpiMessageTag : int32_t {
    TAG_HEARTBEAT = 10,
    TAG_MIGRATION_REQ = 20,
    TAG_MIGRATION_ACK = 21,
    TAG_SENSOR_OBSERVATION = 30,
    TAG_TASK_ANNOUNCE = 40,
    TAG_TASK_BID = 41,
    TAG_TASK_CLAIM = 42,
    TAG_CHECKPOINT_SYNC = 50
};

// Packed migration message structure
struct MigrationPayload {
    DroneState drone_state;
};

// Migration Acknowledgement structure
struct MigrationAckPayload {
    int32_t drone_id{0};
    int32_t source_rank{0};
    int32_t target_rank{0};
    uint32_t seq_num{0};
    bool accepted{true};
};

// Heartbeat message payload
struct HeartbeatPayload {
    int32_t sender_rank{0};
    uint32_t step{0};
    int32_t active_drone_count{0};
    double timestamp_ms{0.0};
};

// Task Auction announcement
struct TaskAuctionPayload {
    int32_t task_id{0};
    TaskType type{TaskType::RECONNAISSANCE};
    Vector3 location;
    int32_t priority{1};
    int32_t announcing_rank{0};
};

// Task Claim confirmation
struct TaskClaimPayload {
    int32_t task_id{0};
    int32_t winning_drone_id{0};
    int32_t winning_rank{0};
    float winning_bid{0.0f};
};

}  // namespace sim
