#include "comm/mpi_manager.hpp"
#include <iostream>
#include <cstring>
#include <chrono>

namespace sim {

MpiManager::MpiManager(int* argc, char*** argv) {
    int provided = 0;
    MPI_Init_thread(argc, argv, MPI_THREAD_FUNNELED, &provided);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank_);
    MPI_Comm_size(MPI_COMM_WORLD, &size_);

    char proc_name[MPI_MAX_PROCESSOR_NAME];
    int name_len = 0;
    MPI_Get_processor_name(proc_name, &name_len);
    hostname_ = std::string(proc_name, name_len);

    std::cout << "[MPI Init] Rank " << rank_ << " / " << size_
              << " running on host: " << hostname_
              << " (Thread level: " << provided << ")\n";
}

MpiManager::~MpiManager() {
    int finalized = 0;
    MPI_Finalized(&finalized);
    if (!finalized) {
        MPI_Finalize();
    }
}

void MpiManager::barrier() {
    if (size_ > 1) {
        MPI_Barrier(MPI_COMM_WORLD);
    }
}

void MpiManager::send_migration_request(int dest_rank, const DroneState& drone) {
    if (size_ <= 1) return;
    MigrationPayload payload{drone};
    MPI_Send(&payload, sizeof(MigrationPayload), MPI_BYTE, dest_rank, TAG_MIGRATION_REQ, MPI_COMM_WORLD);
}

void MpiManager::send_migration_ack(int dest_rank, int drone_id, uint32_t seq, bool accepted) {
    if (size_ <= 1) return;
    MigrationAckPayload ack{drone_id, rank_, dest_rank, seq, accepted};
    MPI_Send(&ack, sizeof(MigrationAckPayload), MPI_BYTE, dest_rank, TAG_MIGRATION_ACK, MPI_COMM_WORLD);
}

void MpiManager::send_heartbeat(int dest_rank, uint32_t step, int active_drones) {
    if (size_ <= 1) return;
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();

    HeartbeatPayload hb{rank_, step, active_drones, static_cast<double>(now)};
    MPI_Send(&hb, sizeof(HeartbeatPayload), MPI_BYTE, dest_rank, TAG_HEARTBEAT, MPI_COMM_WORLD);
}

void MpiManager::send_sensor_observation(int dest_rank, const SensorObservation& obs) {
    if (size_ <= 1) return;
    MPI_Send(&obs, sizeof(SensorObservation), MPI_BYTE, dest_rank, TAG_SENSOR_OBSERVATION, MPI_COMM_WORLD);
}

void MpiManager::poll_incoming_messages(
    std::vector<MigrationPayload>& incoming_migrations,
    std::vector<MigrationAckPayload>& incoming_acks,
    std::vector<HeartbeatPayload>& incoming_heartbeats,
    std::vector<SensorObservation>& incoming_observations
) {
    if (size_ <= 1) return;

    int flag = 0;
    MPI_Status status;

    // Drain all available messages non-blockingly
    while (true) {
        MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &flag, &status);
        if (!flag) {
            break; // No more pending messages right now
        }

        switch (status.MPI_TAG) {
            case TAG_MIGRATION_REQ: {
                MigrationPayload payload;
                MPI_Recv(&payload, sizeof(MigrationPayload), MPI_BYTE, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, &status);
                incoming_migrations.push_back(payload);
                break;
            }
            case TAG_MIGRATION_ACK: {
                MigrationAckPayload ack;
                MPI_Recv(&ack, sizeof(MigrationAckPayload), MPI_BYTE, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, &status);
                incoming_acks.push_back(ack);
                break;
            }
            case TAG_HEARTBEAT: {
                HeartbeatPayload hb;
                MPI_Recv(&hb, sizeof(HeartbeatPayload), MPI_BYTE, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, &status);
                incoming_heartbeats.push_back(hb);
                break;
            }
            case TAG_SENSOR_OBSERVATION: {
                SensorObservation obs;
                MPI_Recv(&obs, sizeof(SensorObservation), MPI_BYTE, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, &status);
                incoming_observations.push_back(obs);
                break;
            }
            default: {
                // Drain unhandled tag to prevent stuck queue
                char discard_buf[512];
                MPI_Recv(discard_buf, sizeof(discard_buf), MPI_BYTE, status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, &status);
                break;
            }
        }
    }
}

}  // namespace sim
