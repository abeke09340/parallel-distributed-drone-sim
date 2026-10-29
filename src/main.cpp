#include "comm/mpi_manager.hpp"
#include "core/config.hpp"
#include "simulation/engine.hpp"
#include "migration/migration_manager.hpp"
#include "fusion/sensor_fusion.hpp"
#include "task_allocation/task_coordinator.hpp"
#include "fault_tolerance/fault_manager.hpp"

#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include <omp.h>

int main(int argc, char** argv) {
    // 1. Initialize Distributed MPI Environment
    sim::MpiManager mpi(&argc, &argv);
    int rank = mpi.get_rank();
    int size = mpi.get_size();
    int peer_rank = mpi.get_peer_rank();

    auto& config = sim::get_global_config();

    // 2. Display Startup Banner
    if (rank == 0) {
        std::cout << "==============================================================\n";
        std::cout << " Distributed Multi-Agent Simulation: Startup\n";
        std::cout << " Group 207 | Abdirakhman Tangirbergen & Meiirlan Meirambek\n";
        std::cout << " Nodes / MPI Ranks: " << size << "\n";
        std::cout << " OpenMP Threads per node: " << omp_get_max_threads() << "\n";
        std::cout << " Drones per node: " << config.drones_per_node << "\n";
        std::cout << "==============================================================\n";
    }

    mpi.barrier();

    // 3. Initialize Core Simulation Engine
    sim::SimulationEngine engine(rank, size);
    engine.initialize_agents(config.drones_per_node);

    // Seed shared simulated targets
    std::vector<sim::Vector3> world_targets = {
        sim::Vector3{250.0f, 500.0f, 0.0f}, // Region A target
        sim::Vector3{490.0f, 510.0f, 0.0f}, // Boundary target
        sim::Vector3{750.0f, 500.0f, 0.0f}  // Region B target
    };
    engine.set_targets(world_targets);

    // 4. Initialize Subsystem Managers
    sim::MigrationManager migration_mgr(engine, mpi);
    sim::SensorFusionManager fusion_mgr(rank);
    sim::TaskCoordinator task_coord(rank, engine);
    sim::FaultManager fault_mgr(rank, peer_rank);

    // Inject an initial mission
    sim::Task mission;
    mission.task_id = (rank * 100) + 1;
    mission.location = (rank == 0) ? sim::Vector3{400.0f, 400.0f, 30.0f} : sim::Vector3{600.0f, 600.0f, 30.0f};
    mission.type = sim::TaskType::RECONNAISSANCE;
    task_coord.submit_task(mission);

    std::cout << "[Node " << rank << "] Initialized "
              << engine.get_drones().size() << " drones in bounds ["
              << engine.get_bounds().min_x << ", " << engine.get_bounds().max_x << "]\n";

    mpi.barrier();

    // 5. Main Simulation Loop
    const int total_steps = 150; // Demo run 150 steps
    auto start_time = std::chrono::high_resolution_clock::now();

    for (int step = 1; step <= total_steps; ++step) {
        // Step A: Parallel update with OpenMP
        engine.step(config.delta_time);

        // Step B: Sensor observations & local fusion
        auto local_obs = engine.collect_sensor_observations();
        fusion_mgr.add_observations(local_obs);

        // Broadcast observations to peer node
        for (const auto& obs : local_obs) {
            mpi.send_sensor_observation(peer_rank, obs);
        }

        // Step C: Detect boundary crossings and trigger outbound migrations
        migration_mgr.process_outbound_migrations();

        // Step D: Send periodic heartbeat
        if (step % config.heartbeat_interval_steps == 0) {
            mpi.send_heartbeat(peer_rank, step, static_cast<int>(engine.get_active_drone_count()));
        }

        // Step E: Non-blocking MPI message polling
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

        // Step F: Handle incoming MPI data
        if (!incoming_migrations.empty()) {
            migration_mgr.handle_incoming_migrations(incoming_migrations);
        }
        if (!incoming_acks.empty()) {
            migration_mgr.handle_incoming_acks(incoming_acks);
        }
        if (!incoming_observations.empty()) {
            fusion_mgr.add_observations(incoming_observations);
        }
        for (const auto& hb : incoming_heartbeats) {
            fault_mgr.record_heartbeat(hb);
        }

        // Step G: Process tasks and decay stale tracks
        task_coord.process_pending_tasks();
        fusion_mgr.decay_tracks(step);
        fault_mgr.check_liveness(step);

        // Step H: Telemetry log every 30 steps
        if (step % 30 == 0 || step == total_steps) {
            std::cout << "[Step " << step << " | Node " << rank << "] "
                      << "Active Drones: " << engine.get_active_drone_count()
                      << " | Migrated Out: " << migration_mgr.get_total_migrated_out()
                      << " | Migrated In: " << migration_mgr.get_total_migrated_in()
                      << " | Fused Tracks: " << fusion_mgr.get_track_count()
                      << "\n";
        }

        // Pace simulation to mimic 20 Hz
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    mpi.barrier();

    if (rank == 0) {
        std::cout << "\n==============================================================\n";
        std::cout << " Simulation Completed Successfully!\n";
        std::cout << " Total Time: " << total_ms << " ms (" << (total_ms / total_steps) << " ms/step)\n";
        std::cout << "==============================================================\n";
    }

    return 0;
}
