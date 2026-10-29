#pragma once

#include "core/types.hpp"
#include "comm/protocol.hpp"
#include <vector>

namespace sim {

enum class NodeHealthState {
    HEALTHY,
    DEGRADED,
    PARTITIONED,
    FAILED
};

class FaultManager {
public:
    FaultManager(int rank, int peer_rank);

    // Record incoming heartbeat
    void record_heartbeat(const HeartbeatPayload& hb);

    // Check heartbeat expiration at current step
    void check_liveness(uint32_t current_step);

    NodeHealthState get_peer_state() const { return peer_state_; }
    uint32_t get_last_peer_step() const { return last_peer_step_; }

private:
    int rank_{0};
    int peer_rank_{1};
    uint32_t last_heartbeat_step_{0};
    uint32_t last_peer_step_{0};
    NodeHealthState peer_state_{NodeHealthState::HEALTHY};
};

}  // namespace sim
