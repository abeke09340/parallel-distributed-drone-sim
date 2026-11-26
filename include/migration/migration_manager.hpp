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

    // Process local drones that crossed region bounds (with hysteresis)
    void process_outbound_migrations();

    // Process incoming migration requests from remote node (with de-duplication)
    void handle_incoming_migrations(const std::vector<MigrationPayload>& incoming);

    // Process incoming migration ACKs from remote node
    void handle_incoming_acks(const std::vector<MigrationAckPayload>& acks);

    // Roll back pending migrations that exceeded ACK timeout
    void check_timeouts(uint32_t current_step, uint32_t timeout_steps = 20);

    // Get statistics
    size_t get_total_migrated_out() const { return total_migrated_out_; }
    size_t get_total_migrated_in() const { return total_migrated_in_; }
    size_t get_total_rollbacks() const { return total_rollbacks_; }
    size_t get_pending_count() const { return pending_acks_.size(); }

private:
    struct PendingMigration {
        DroneState state;
        uint32_t initiated_step{0};
    };

    SimulationEngine& engine_;
    MpiManager& mpi_;

    // Pending ACKs: drone_id -> PendingMigration (for seq verification & rollback)
    std::unordered_map<int32_t, PendingMigration> pending_acks_;

    size_t total_migrated_out_{0};
    size_t total_migrated_in_{0};
    size_t total_rollbacks_{0};
};

}  // namespace sim
