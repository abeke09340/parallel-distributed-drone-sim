#pragma once

#include "comm/protocol.hpp"
#include <cstdint>
#include <list>
#include <mpi.h>
#include <string>
#include <vector>

namespace sim {

class MpiManager {
public:
    MpiManager(int* argc, char*** argv);
    ~MpiManager();

    // Prevent copies
    MpiManager(const MpiManager&) = delete;
    MpiManager& operator=(const MpiManager&) = delete;

    int get_rank() const { return rank_; }
    int get_size() const { return size_; }
    int get_peer_rank() const { return (rank_ == 0) ? 1 : 0; }
    const std::string& get_hostname() const { return hostname_; }
    size_t get_pending_send_count() const { return pending_sends_.size(); }

    // Non-blocking send methods (MPI_Isend)
    void send_migration_request(int dest_rank, const DroneState& drone);
    void send_migration_batch(int dest_rank, const std::vector<DroneState>& drones);
    void send_migration_ack(int dest_rank, int drone_id, uint32_t seq, bool accepted);
    void send_heartbeat(int dest_rank, uint32_t step, int active_drones);
    void send_sensor_observation(int dest_rank, const SensorObservation& obs);
    void send_sensor_observations_batch(int dest_rank, const std::vector<SensorObservation>& observations);
    void send_combat_event(int dest_rank, const CombatEventPayload& evt);

    // Reclaim completed non-blocking send buffers
    void flush_completed_sends();

    // Non-blocking poll for incoming MPI messages (supports single & batched payloads)
    void poll_incoming_messages(
        std::vector<MigrationPayload>& incoming_migrations,
        std::vector<MigrationAckPayload>& incoming_acks,
        std::vector<HeartbeatPayload>& incoming_heartbeats,
        std::vector<SensorObservation>& incoming_observations,
        std::vector<CombatEventPayload>* incoming_combat_events = nullptr
    );

    // Barrier sync
    void barrier();

private:
    struct PendingSend {
        MPI_Request request{MPI_REQUEST_NULL};
        std::vector<uint8_t> buffer;
    };

    void post_isend(int dest_rank, MpiMessageTag tag, const void* data, size_t bytes);

    int rank_{0};
    int size_{1};
    std::string hostname_;
    std::list<PendingSend> pending_sends_;
};

}  // namespace sim
