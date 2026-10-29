#include "fault_tolerance/fault_manager.hpp"
#include "core/config.hpp"
#include <iostream>

namespace sim {

FaultManager::FaultManager(int rank, int peer_rank)
    : rank_(rank), peer_rank_(peer_rank) {}

void FaultManager::record_heartbeat(const HeartbeatPayload& hb) {
    last_heartbeat_step_ = hb.step;
    last_peer_step_ = hb.step;
    peer_state_ = NodeHealthState::HEALTHY;
}

void FaultManager::check_liveness(uint32_t current_step) {
    const auto& config = get_global_config();
    uint32_t elapsed = (current_step > last_heartbeat_step_) ? (current_step - last_heartbeat_step_) : 0;

    if (elapsed > static_cast<uint32_t>(config.heartbeat_timeout_steps)) {
        if (peer_state_ == NodeHealthState::HEALTHY) {
            peer_state_ = NodeHealthState::FAILED;
            std::cerr << "[Fault Alert] Node " << rank_
                      << ": Peer Node " << peer_rank_
                      << " is UNREACHABLE (Missed " << elapsed << " steps) -> Entering autonomous mode!\n";
        }
    }
}

}  // namespace sim
