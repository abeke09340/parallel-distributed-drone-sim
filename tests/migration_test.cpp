#include "comm/mpi_manager.hpp"
#include "core/config.hpp"
#include "migration/migration_manager.hpp"
#include "simulation/engine.hpp"

#include <chrono>
#include <iostream>
#include <mpi.h>
#include <numeric>
#include <set>
#include <vector>

int main(int argc, char** argv) {
    sim::MpiManager mpi(&argc, &argv);
    const int rank = mpi.get_rank();
    const int size = mpi.get_size();
    const int peer_rank = mpi.get_peer_rank();

    if (size != 2) {
        if (rank == 0) {
            std::cerr << "[FAIL] migration_test requires exactly 2 MPI ranks (mpirun -np 2).\n";
        }
        return 1;
    }

    auto& config = sim::get_global_config();
    const int drones_per_node = config.drones_per_node;  // 500 per node -> 1000 total
    const int expected_total_drones = drones_per_node * size;

    sim::SimulationEngine engine(rank, size);
    engine.initialize_agents(drones_per_node);

    // Force a deterministic subset of drones on each node to fly directly across X = 25000 +/- 250 m
    auto& drones = engine.get_mutable_drones();
    for (int i = 0; i < 50 && i < static_cast<int>(drones.size()); ++i) {
        auto& st = drones[static_cast<size_t>(i)].get_mutable_state();
        if (rank == 0) {
            st.position = sim::Vector3{24900.0f, 1000.0f + i * 200.0f, 100.0f};
            st.target_waypoint = sim::Vector3{26500.0f, st.position.y, 100.0f};
        } else {
            st.position = sim::Vector3{25100.0f, 1000.0f + i * 200.0f, 100.0f};
            st.target_waypoint = sim::Vector3{23500.0f, st.position.y, 100.0f};
        }
        st.hp = 85.0f;
        st.threat_level = 0.25f;
    }

    sim::MigrationManager migration_mgr(engine, mpi);
    mpi.barrier();

    const int total_steps = 200;
    double total_migration_poll_us = 0.0;
    int migration_steps = 0;

    for (int step = 1; step <= total_steps; ++step) {
        engine.step(config.delta_time);

        auto t0 = std::chrono::high_resolution_clock::now();
        migration_mgr.process_outbound_migrations();

        std::vector<sim::MigrationPayload> incoming_migrations;
        std::vector<sim::MigrationAckPayload> incoming_acks;
        std::vector<sim::HeartbeatPayload> incoming_heartbeats;
        std::vector<sim::SensorObservation> incoming_observations;

        mpi.poll_incoming_messages(
            incoming_migrations,
            incoming_acks,
            incoming_heartbeats,
            incoming_observations
        );

        if (!incoming_migrations.empty()) {
            migration_mgr.handle_incoming_migrations(incoming_migrations);
        }
        if (!incoming_acks.empty()) {
            migration_mgr.handle_incoming_acks(incoming_acks);
        }
        migration_mgr.check_timeouts(static_cast<uint32_t>(step));
        auto t1 = std::chrono::high_resolution_clock::now();

        if (!incoming_migrations.empty() || !incoming_acks.empty()) {
            total_migration_poll_us +=
                std::chrono::duration<double, std::micro>(t1 - t0).count();
            migration_steps++;
        }
    }

    // Drain any final in-flight ACKs across a barrier
    mpi.barrier();
    for (int flush_step = 0; flush_step < 5; ++flush_step) {
        std::vector<sim::MigrationPayload> m;
        std::vector<sim::MigrationAckPayload> a;
        std::vector<sim::HeartbeatPayload> h;
        std::vector<sim::SensorObservation> o;
        mpi.poll_incoming_messages(m, a, h, o);
        if (!m.empty()) migration_mgr.handle_incoming_migrations(m);
        if (!a.empty()) migration_mgr.handle_incoming_acks(a);
        mpi.barrier();
    }

    // Gather all local drone IDs across both ranks to verify ZERO duplicates and ZERO losses
    std::vector<int32_t> local_ids;
    local_ids.reserve(engine.get_drones().size());
    for (const auto& d : engine.get_drones()) {
        local_ids.push_back(d.get_id());
    }

    int local_count = static_cast<int>(local_ids.size());
    int global_count = 0;
    MPI_Allreduce(&local_count, &global_count, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);

    int counts[2] = {0, 0};
    MPI_Gather(&local_count, 1, MPI_INT, counts, 1, MPI_INT, 0, MPI_COMM_WORLD);

    std::vector<int32_t> all_ids;
    std::vector<int> displs(2, 0);
    if (rank == 0) {
        all_ids.resize(static_cast<size_t>(counts[0] + counts[1]));
        displs[1] = counts[0];
    }
    MPI_Gatherv(
        local_ids.data(), local_count, MPI_INT32_T,
        all_ids.data(), counts, displs.data(), MPI_INT32_T,
        0, MPI_COMM_WORLD
    );

    uint64_t out_count = static_cast<uint64_t>(migration_mgr.get_total_migrated_out());
    uint64_t in_count = static_cast<uint64_t>(migration_mgr.get_total_migrated_in());
    uint64_t peer_in_count = 0;
    MPI_Sendrecv(
        &in_count, 1, MPI_UINT64_T, peer_rank, 99,
        &peer_in_count, 1, MPI_UINT64_T, peer_rank, 99,
        MPI_COMM_WORLD, MPI_STATUS_IGNORE
    );

    bool passed = true;
    if (global_count != expected_total_drones) passed = false;
    if (out_count == 0 || out_count != peer_in_count) passed = false;
    if (migration_mgr.get_pending_count() != 0) {
        passed = false;
    }

    if (rank == 0) {
        std::set<int32_t> unique_ids(all_ids.begin(), all_ids.end());
        if (static_cast<int>(unique_ids.size()) != expected_total_drones) {
            passed = false;
        }
        double avg_mig_us = (migration_steps > 0)
                                ? (total_migration_poll_us / static_cast<double>(migration_steps))
                                : 0.0;

        std::cout << "\n==============================================================\n";
        std::cout << " Phase 2 Migration & Zero-Duplicate Verification Test\n";
        std::cout << "==============================================================\n";
        std::cout << " Expected Total Drones : " << expected_total_drones << "\n";
        std::cout << " Actual Total Drones   : " << global_count << " (Rank 0: " << counts[0]
                  << ", Rank 1: " << counts[1] << ")\n";
        std::cout << " Unique Drone IDs      : " << unique_ids.size() << "\n";
        std::cout << " Rank 0 Out / Rank 1 In: " << out_count << " / " << peer_in_count << "\n";
        std::cout << " Avg Migration Poll    : " << avg_mig_us << " us/event-step\n";
        std::cout << " Pending / Rollbacks   : " << migration_mgr.get_pending_count()
                  << " / " << migration_mgr.get_total_rollbacks() << "\n";
        std::cout << " Status                : " << (passed ? "PASSED" : "FAILED") << "\n";
        std::cout << "==============================================================\n";
    }

    int local_pass = passed ? 1 : 0;
    int global_pass = 0;
    MPI_Allreduce(&local_pass, &global_pass, 1, MPI_INT, MPI_MIN, MPI_COMM_WORLD);
    return (global_pass == 1) ? 0 : 1;
}
