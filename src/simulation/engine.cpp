#include "simulation/engine.hpp"
#include "core/config.hpp"
#include <omp.h>
#include <random>
#include <iostream>
#include <algorithm>

namespace sim {

SimulationEngine::SimulationEngine(int rank, int num_ranks)
    : rank_(rank), num_ranks_(num_ranks) {
    const auto& config = get_global_config();
    bounds_ = config.get_region_bounds_for_rank(rank, num_ranks);

    if (config.openmp_threads > 0) {
        omp_set_num_threads(config.openmp_threads);
    }
}

void SimulationEngine::initialize_agents(int count) {
    drones_.clear();
    drones_.reserve(count);

    std::mt19937 rng(1337 + rank_ * 42);
    std::uniform_real_distribution<float> dist_x(bounds_.min_x + 20.0f, bounds_.max_x - 20.0f);
    std::uniform_real_distribution<float> dist_y(bounds_.min_y + 20.0f, bounds_.max_y - 20.0f);
    std::uniform_real_distribution<float> dist_z(10.0f, bounds_.max_z - 10.0f);

    // Random targets for patrol across regions
    const auto& config = get_global_config();
    std::uniform_real_distribution<float> wp_x(0.0f, config.world_width);
    std::uniform_real_distribution<float> wp_y(0.0f, config.world_length);

    int id_offset = rank_ * 10000;
    for (int i = 0; i < count; ++i) {
        Vector3 pos{dist_x(rng), dist_y(rng), dist_z(rng)};
        Drone drone(id_offset + i, rank_, pos);
        // Set an initial waypoint, possibly towards the opposite region
        drone.set_waypoint(Vector3{wp_x(rng), wp_y(rng), pos.z});
        drones_.push_back(drone);
    }
}

void SimulationEngine::set_targets(const std::vector<Vector3>& targets) {
    targets_ = targets;
}

void SimulationEngine::step(float dt) {
    current_step_++;

    // Parallel agent kinematic updates with OpenMP
    #pragma omp parallel for schedule(static)
    for (size_t i = 0; i < drones_.size(); ++i) {
        drones_[i].update(dt);
    }

    // Parallel sensor scans with thread-local buffers
    latest_observations_.clear();
    std::vector<std::vector<SensorObservation>> thread_observations;

    #pragma omp parallel
    {
        #pragma omp single
        {
            thread_observations.resize(omp_get_num_threads());
        }

        int tid = omp_get_thread_num();

        #pragma omp for schedule(dynamic, 16)
        for (size_t i = 0; i < drones_.size(); ++i) {
            auto obs = drones_[i].scan_targets(targets_, current_step_, 100.0f);
            if (!obs.empty()) {
                thread_observations[tid].insert(
                    thread_observations[tid].end(), obs.begin(), obs.end()
                );
            }
        }
    }

    // Aggregate thread-local observations
    for (const auto& obs_vec : thread_observations) {
        latest_observations_.insert(latest_observations_.end(), obs_vec.begin(), obs_vec.end());
    }
}

std::vector<DroneState> SimulationEngine::extract_boundary_crossing_drones() {
    std::vector<DroneState> migrating_out;

    for (auto& drone : drones_) {
        if (drone.get_status() == DroneStatus::PATROL) {
            Vector3 pos = drone.get_position();
            // Check if drone has moved outside this node's spatial domain
            if (!bounds_.contains(pos)) {
                drone.set_status(DroneStatus::MIGRATING_OUT);
                migrating_out.push_back(drone.get_state());
            }
        }
    }

    return migrating_out;
}

void SimulationEngine::ingest_migrated_drone(const DroneState& state) {
    // Check if drone is already locally present to avoid duplicates
    for (auto& drone : drones_) {
        if (drone.get_id() == state.id) {
            drone = Drone(state);
            drone.set_status(DroneStatus::PATROL);
            drone.get_mutable_state().owner_rank = rank_;
            return;
        }
    }

    Drone new_drone(state);
    new_drone.set_status(DroneStatus::PATROL);
    new_drone.get_mutable_state().owner_rank = rank_;
    drones_.push_back(new_drone);
}

void SimulationEngine::finalize_migrated_out(int drone_id) {
    // Remove the drone that successfully migrated to the other node
    drones_.erase(
        std::remove_if(drones_.begin(), drones_.end(),
            [drone_id](const Drone& d) {
                return d.get_id() == drone_id;
            }),
        drones_.end()
    );
}

std::vector<SensorObservation> SimulationEngine::collect_sensor_observations() {
    return latest_observations_;
}

size_t SimulationEngine::get_active_drone_count() const {
    size_t active = 0;
    for (const auto& d : drones_) {
        if (d.get_status() == DroneStatus::PATROL || d.get_status() == DroneStatus::TASK_EXECUTING) {
            active++;
        }
    }
    return active;
}

}  // namespace sim
