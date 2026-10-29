#pragma once

#include "core/types.hpp"
#include "comm/mpi_manager.hpp"
#include "simulation/engine.hpp"
#include <unordered_map>
#include <vector>

namespace sim {

class MigrationManager {
public:
    MigrationManager(SimulationEngine& engine, MpiManager& mpi);

    // Process local drones that crossed region bounds
    void process_outbound_migrations();

    // Process incoming migration requests from remote node
    void handle_incoming_migrations(const std::vector<MigrationPayload>& incoming);

    // Process incoming migration ACKs from remote node
    void handle_incoming_acks(const std::vector<MigrationAckPayload>& acks);

    // Get statistics
    size_t get_total_migrated_out() const { return total_migrated_out_; }
    size_t get_total_migrated_in() const { return total_migrated_in_; }

private:
    SimulationEngine& engine_;
    MpiManager& mpi_;

    // Pending acks: drone_id -> DroneState (in case retransmission needed)
    std::unordered_map<int32_t, DroneState> pending_acks_;

    size_t total_migrated_out_{0};
    size_t total_migrated_in_{0};
};

}  // namespace sim
