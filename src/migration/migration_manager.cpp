#include "migration/migration_manager.hpp"
#include <iostream>

namespace sim {

MigrationManager::MigrationManager(SimulationEngine& engine, MpiManager& mpi)
    : engine_(engine), mpi_(mpi) {}

void MigrationManager::process_outbound_migrations() {
    auto crossing_drones = engine_.extract_boundary_crossing_drones();
    if (crossing_drones.empty()) return;

    int peer = mpi_.get_peer_rank();

    for (const auto& drone_state : crossing_drones) {
        // Record in pending ACKs
        pending_acks_[drone_state.id] = drone_state;

        // Transmit state across MPI
        mpi_.send_migration_request(peer, drone_state);

        std::cout << "[Migration] Rank " << engine_.get_rank()
                  << " initiating migration for Drone #" << drone_state.id
                  << " -> Node " << peer
                  << " at pos (" << drone_state.position.x << ", "
                  << drone_state.position.y << ")\n";
    }
}

void MigrationManager::handle_incoming_migrations(const std::vector<MigrationPayload>& incoming) {
    for (const auto& payload : incoming) {
        const auto& state = payload.drone_state;

        // Ingest into local engine
        engine_.ingest_migrated_drone(state);
        total_migrated_in_++;

        // Send ACK back to source node
        mpi_.send_migration_ack(state.owner_rank, state.id, state.seq_num, true);

        std::cout << "[Migration] Rank " << engine_.get_rank()
                  << " ACCEPTED Drone #" << state.id
                  << " from Rank " << state.owner_rank
                  << " (Total migrated in: " << total_migrated_in_ << ")\n";
    }
}

void MigrationManager::handle_incoming_acks(const std::vector<MigrationAckPayload>& acks) {
    for (const auto& ack : acks) {
        if (ack.accepted) {
            auto it = pending_acks_.find(ack.drone_id);
            if (it != pending_acks_.end()) {
                // Safely purge local copy now that peer has taken ownership
                engine_.finalize_migrated_out(ack.drone_id);
                pending_acks_.erase(it);
                total_migrated_out_++;

                std::cout << "[Migration] Rank " << engine_.get_rank()
                          << " FINALIZED migration for Drone #" << ack.drone_id
                          << " (Ownership transferred cleanly)\n";
            }
        }
    }
}

}  // namespace sim
