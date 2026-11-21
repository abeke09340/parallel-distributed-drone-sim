#include "comm/mpi_manager.hpp"
#include <chrono>
#include <cstring>
#include <iostream>

namespace sim {

MpiManager::MpiManager(int* argc, char*** argv) {
    int provided = 0;
    MPI_Init_thread(argc, argv, MPI_THREAD_FUNNELED, &provided);
    MPI_Comm_rank(MPI_COMM_WORLD, &rank_);
    MPI_Comm_size(MPI_COMM_WORLD, &size_);

    char proc_name[MPI_MAX_PROCESSOR_NAME];
    int name_len = 0;
    MPI_Get_processor_name(proc_name, &name_len);
    hostname_ = std::string(proc_name, static_cast<size_t>(name_len));

    std::cout << "[MPI Init] Rank " << rank_ << " / " << size_
              << " running on host: " << hostname_
              << " (Thread level: " << provided << ")\n";
}

MpiManager::~MpiManager() {
    // Ensure all pending non-blocking sends complete before MPI_Finalize
    for (auto& pending : pending_sends_) {
        if (pending.request != MPI_REQUEST_NULL) {
            MPI_Wait(&pending.request, MPI_STATUS_IGNORE);
        }
    }
    pending_sends_.clear();

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

void MpiManager::flush_completed_sends() {
    for (auto it = pending_sends_.begin(); it != pending_sends_.end();) {
        int completed = 0;
        MPI_Test(&it->request, &completed, MPI_STATUS_IGNORE);
        if (completed) {
            it = pending_sends_.erase(it);
        } else {
            ++it;
        }
    }
}

void MpiManager::post_isend(int dest_rank, MpiMessageTag tag, const void* data, size_t bytes) {
    if (size_ <= 1 || bytes == 0) {
        return;
    }

    flush_completed_sends();

    PendingSend& entry = pending_sends_.emplace_back();
    entry.buffer.resize(bytes);
    std::memcpy(entry.buffer.data(), data, bytes);

    MPI_Isend(
        entry.buffer.data(),
        static_cast<int>(bytes),
        MPI_BYTE,
        dest_rank,
        tag,
        MPI_COMM_WORLD,
        &entry.request
    );
}

void MpiManager::send_migration_request(int dest_rank, const DroneState& drone) {
    MigrationPayload payload{drone};
    post_isend(dest_rank, TAG_MIGRATION_REQ, &payload, sizeof(MigrationPayload));
}

void MpiManager::send_migration_batch(int dest_rank, const std::vector<DroneState>& drones) {
    if (drones.empty()) return;
    std::vector<MigrationPayload> payloads;
    payloads.reserve(drones.size());
    for (const auto& d : drones) {
        payloads.push_back(MigrationPayload{d});
    }
    post_isend(
        dest_rank,
        TAG_MIGRATION_REQ,
        payloads.data(),
        payloads.size() * sizeof(MigrationPayload)
    );
}

void MpiManager::send_migration_ack(int dest_rank, int drone_id, uint32_t seq, bool accepted) {
    MigrationAckPayload ack{drone_id, rank_, dest_rank, seq, accepted};
    post_isend(dest_rank, TAG_MIGRATION_ACK, &ack, sizeof(MigrationAckPayload));
}

void MpiManager::send_heartbeat(int dest_rank, uint32_t step, int active_drones) {
    auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()
    ).count();

    HeartbeatPayload hb{rank_, step, active_drones, static_cast<double>(now)};
    post_isend(dest_rank, TAG_HEARTBEAT, &hb, sizeof(HeartbeatPayload));
}

void MpiManager::send_sensor_observation(int dest_rank, const SensorObservation& obs) {
    post_isend(dest_rank, TAG_SENSOR_OBSERVATION, &obs, sizeof(SensorObservation));
}

void MpiManager::send_sensor_observations_batch(
    int dest_rank,
    const std::vector<SensorObservation>& observations
) {
    if (observations.empty()) return;
    post_isend(
        dest_rank,
        TAG_SENSOR_OBSERVATION,
        observations.data(),
        observations.size() * sizeof(SensorObservation)
    );
}

void MpiManager::send_combat_event(int dest_rank, const CombatEventPayload& evt) {
    post_isend(dest_rank, TAG_COMBAT_EVENT, &evt, sizeof(CombatEventPayload));
}

namespace {
template <typename T>
void recv_typed_batch(MPI_Status& probe_status, std::vector<T>& out) {
    int byte_count = 0;
    MPI_Get_count(&probe_status, MPI_BYTE, &byte_count);
    if (byte_count <= 0) return;

    size_t elem_count = static_cast<size_t>(byte_count) / sizeof(T);
    if (elem_count == 0) {
        std::vector<uint8_t> discard(static_cast<size_t>(byte_count));
        MPI_Recv(discard.data(), byte_count, MPI_BYTE,
                 probe_status.MPI_SOURCE, probe_status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        return;
    }

    size_t old_size = out.size();
    out.resize(old_size + elem_count);
    MPI_Recv(out.data() + old_size, static_cast<int>(elem_count * sizeof(T)), MPI_BYTE,
             probe_status.MPI_SOURCE, probe_status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
}
}  // namespace

void MpiManager::poll_incoming_messages(
    std::vector<MigrationPayload>& incoming_migrations,
    std::vector<MigrationAckPayload>& incoming_acks,
    std::vector<HeartbeatPayload>& incoming_heartbeats,
    std::vector<SensorObservation>& incoming_observations,
    std::vector<CombatEventPayload>* incoming_combat_events
) {
    if (size_ <= 1) return;

    flush_completed_sends();

    int flag = 0;
    MPI_Status status;

    // Drain all available messages non-blockingly
    while (true) {
        MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &flag, &status);
        if (!flag) {
            break;  // No more pending messages right now
        }

        switch (status.MPI_TAG) {
            case TAG_MIGRATION_REQ:
                recv_typed_batch(status, incoming_migrations);
                break;
            case TAG_MIGRATION_ACK:
                recv_typed_batch(status, incoming_acks);
                break;
            case TAG_HEARTBEAT:
                recv_typed_batch(status, incoming_heartbeats);
                break;
            case TAG_SENSOR_OBSERVATION:
                recv_typed_batch(status, incoming_observations);
                break;
            case TAG_COMBAT_EVENT:
                if (incoming_combat_events != nullptr) {
                    recv_typed_batch(status, *incoming_combat_events);
                } else {
                    int byte_count = 0;
                    MPI_Get_count(&status, MPI_BYTE, &byte_count);
                    std::vector<uint8_t> discard(static_cast<size_t>(std::max(0, byte_count)));
                    MPI_Recv(discard.data(), byte_count, MPI_BYTE,
                             status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                }
                break;
            default: {
                int byte_count = 0;
                MPI_Get_count(&status, MPI_BYTE, &byte_count);
                std::vector<uint8_t> discard(static_cast<size_t>(std::max(0, byte_count)));
                MPI_Recv(discard.data(), byte_count, MPI_BYTE,
                         status.MPI_SOURCE, status.MPI_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
                break;
            }
        }
    }
}

}  // namespace sim
