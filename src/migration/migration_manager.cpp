#include "migration/migration_manager.hpp"
#include <iostream>

namespace sim {

MigrationManager::MigrationManager(SimulationEngine& engine, MpiManager& mpi)
    : engine_(engine), mpi_(mpi) {}

void MigrationManager::process_outbound_migrations() {
    auto crossing_drones = engine_.extract_boundary_crossing_drones();
    if (crossing_drones.empty()) return;

    int peer = mpi_.get_peer_rank();
    uint32_t step = engine_.get_current_step();

    for (const auto& drone_state : crossing_drones) {
        pending_acks_[drone_state.id] = PendingMigration{drone_state, step};
    }

    // Batch-transmit all boundary-crossing drones in a single non-blocking MPI_Isend
    mpi_.send_migration_batch(peer, crossing_drones);

    std::cout << "[Migration] Rank " << engine_.get_rank()
              << " initiated batch migration for " << crossing_drones.size()
              << " drone(s) -> Node " << peer << " at step " << step << "\n";
}

void MigrationManager::handle_incoming_migrations(const std::vector<MigrationPayload>& incoming) {
    for (const auto& payload : incoming) {
        const auto& state = payload.drone_state;

        // Ingest into local engine (returns false if duplicate)
        bool newly_ingested = engine_.ingest_migrated_drone(state);
        if (newly_ingested) {
            total_migrated_in_++;
        }

        // Send ACK back to source node regardless, so sender can finalize cleanly
        mpi_.send_migration_ack(state.owner_rank, state.id, state.seq_num, true);

        if (newly_ingested) {
            std::cout << "[Migration] Rank " << engine_.get_rank()
                      << " ACCEPTED Drone #" << state.id
                      << " (hp=" << state.hp << ", threat=" << state.threat_level
                      << ") from Rank " << state.owner_rank
                      << " (Total migrated in: " << total_migrated_in_ << ")\n";
        }
    }
}

void MigrationManager::handle_incoming_acks(const std::vector<MigrationAckPayload>& acks) {
    for (const auto& ack : acks) {
        auto it = pending_acks_.find(ack.drone_id);
        if (it == pending_acks_.end()) {
            continue;
        }

        // Ignore stale ACKs from earlier migration cycles
        if (ack.seq_num != it->second.state.seq_num) {
            continue;
        }

        if (ack.accepted) {
            // Safely purge local copy now that peer has taken ownership
            engine_.finalize_migrated_out(ack.drone_id);
            pending_acks_.erase(it);
            total_migrated_out_++;

            std::cout << "[Migration] Rank " << engine_.get_rank()
                      << " FINALIZED migration for Drone #" << ack.drone_id
                      << " (Ownership transferred cleanly)\n";
        } else {
            // Peer rejected migration -> roll back local drone to active state
            engine_.rollback_migrated_out(ack.drone_id);
            pending_acks_.erase(it);
            total_rollbacks_++;

            std::cerr << "[Migration] Rank " << engine_.get_rank()
                      << " ROLLED BACK rejected migration for Drone #" << ack.drone_id << "\n";
        }
    }
}

void MigrationManager::check_timeouts(uint32_t current_step, uint32_t timeout_steps) {
    for (auto it = pending_acks_.begin(); it != pending_acks_.end();) {
        uint32_t elapsed = (current_step >= it->second.initiated_step)
                               ? (current_step - it->second.initiated_step)
                               : 0;
        if (elapsed >= timeout_steps) {
            int32_t drone_id = it->first;
            engine_.rollback_migrated_out(drone_id);
            total_rollbacks_++;
            std::cerr << "[Migration Timeout] Rank " << engine_.get_rank()
                      << " rolled back Drone #" << drone_id
                      << " after " << elapsed << " steps without ACK\n";
            it = pending_acks_.erase(it);
        } else {
            ++it;
        }
    }
}

}  // namespace sim
