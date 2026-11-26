#pragma once

#include "core/types.hpp"
#include <type_traits>

namespace sim {

// MPI message tags for communication multiplexing
enum MpiMessageTag : int32_t {
    TAG_HEARTBEAT = 10,
    TAG_MIGRATION_REQ = 20,
    TAG_MIGRATION_ACK = 21,
    TAG_SENSOR_OBSERVATION = 30,
    TAG_COMBAT_EVENT = 35,
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

// Cross-node combat hit synchronization payload
struct CombatEventPayload {
    int32_t target_id{0};
    int32_t attacker_drone_id{0};
    int32_t source_rank{0};
    uint32_t step{0};
    float damage{50.0f};
    bool hit{true};
};

// Task Auction announcement
struct TaskAuctionPayload {
    int32_t task_id{0};
    TaskType type{TaskType::RECONNAISSANCE};
    Vector3 location;
    int32_t target_id{-1};
    int32_t priority{1};
    int32_t announcing_rank{0};
    int32_t remote_deficit{1};
};

// Task Auction bid response
struct TaskBidPayload {
    int32_t task_id{0};
    int32_t target_id{-1};
    int32_t candidate_drone_id{-1};
    int32_t bidder_rank{0};
    int32_t announcing_rank{0};
    float utility{0.0f};
};

// Task Claim confirmation
struct TaskClaimPayload {
    int32_t task_id{0};
    int32_t target_id{-1};
    int32_t winning_drone_id{0};
    int32_t winning_rank{0};
    float winning_bid{0.0f};
    Vector3 location;
};

static_assert(std::is_trivially_copyable_v<MigrationPayload>,
              "MigrationPayload must be trivially copyable for MPI");
static_assert(std::is_trivially_copyable_v<MigrationAckPayload>,
              "MigrationAckPayload must be trivially copyable for MPI");
static_assert(std::is_trivially_copyable_v<HeartbeatPayload>,
              "HeartbeatPayload must be trivially copyable for MPI");
static_assert(std::is_trivially_copyable_v<CombatEventPayload>,
              "CombatEventPayload must be trivially copyable for MPI");
static_assert(std::is_trivially_copyable_v<TaskAuctionPayload>,
              "TaskAuctionPayload must be trivially copyable for MPI");
static_assert(std::is_trivially_copyable_v<TaskBidPayload>,
              "TaskBidPayload must be trivially copyable for MPI");
static_assert(std::is_trivially_copyable_v<TaskClaimPayload>,
              "TaskClaimPayload must be trivially copyable for MPI");

}  // namespace sim
