#pragma once

#include "comm/protocol.hpp"
#include <mpi.h>
#include <vector>
#include <string>

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

    // Send migration request
    void send_migration_request(int dest_rank, const DroneState& drone);

    // Send migration ACK
    void send_migration_ack(int dest_rank, int drone_id, uint32_t seq, bool accepted);

    // Send heartbeat
    void send_heartbeat(int dest_rank, uint32_t step, int active_drones);

    // Send sensor observation
    void send_sensor_observation(int dest_rank, const SensorObservation& obs);

    // Non-blocking poll for incoming MPI messages
    void poll_incoming_messages(
        std::vector<MigrationPayload>& incoming_migrations,
        std::vector<MigrationAckPayload>& incoming_acks,
        std::vector<HeartbeatPayload>& incoming_heartbeats,
        std::vector<SensorObservation>& incoming_observations
    );

    // Barrier sync
    void barrier();

private:
    int rank_{0};
    int size_{1};
    std::string hostname_;
};

}  // namespace sim
