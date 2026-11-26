#include "simulation/engine.hpp"
#include "core/config.hpp"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <omp.h>
#include <random>

namespace sim {

namespace {
bool is_active_fsm_status(DroneStatus status) {
    return status == DroneStatus::PATROL ||
           status == DroneStatus::TASK_EXECUTING ||
           status == DroneStatus::RECONNAISSANCE ||
           status == DroneStatus::OBJECT_INSPECTION ||
           status == DroneStatus::TARGET_ELIMINATION ||
           status == DroneStatus::EVADE;
}
}  // namespace

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
    current_step_ = 0;

    std::mt19937 rng(1337 + rank_ * 42);
    std::uniform_real_distribution<float> dist_x(bounds_.min_x + 20.0f, bounds_.max_x - 20.0f);
    std::uniform_real_distribution<float> dist_y(bounds_.min_y + 20.0f, bounds_.max_y - 20.0f);
    std::uniform_real_distribution<float> dist_z(10.0f, bounds_.max_z - 10.0f);

    // Random targets for patrol across regions
    const auto& config = get_global_config();
    std::uniform_real_distribution<float> wp_x(20.0f, config.world_width - 20.0f);
    std::uniform_real_distribution<float> wp_y(20.0f, config.world_length - 20.0f);

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
    target_entities_.clear();
}

void SimulationEngine::set_target_entities(const std::vector<TargetEntity>& entities) {
    target_entities_ = entities;
    targets_.clear();
    targets_.reserve(entities.size());
    for (const auto& e : entities) {
        targets_.push_back(e.position);
    }
}

void SimulationEngine::apply_damage_event(const DamageEvent& evt) {
    if (!evt.hit || evt.damage <= 0.0f) return;
    for (auto& target : target_entities_) {
        if (target.id == evt.target_id && target.alive) {
            target.hp = std::max(0.0f, target.hp - evt.damage);
            if (target.hp <= 0.0f) {
                target.alive = false;
            }
            break;
        }
    }
}

void SimulationEngine::step(float dt) {
    current_step_++;

    const auto& config = get_global_config();
    const std::ptrdiff_t drone_count = static_cast<std::ptrdiff_t>(drones_.size());
    const bool use_entities = !target_entities_.empty();

    // Parallel agent kinematic & threat/FSM updates with OpenMP (signed index for MSVC compatibility)
    #pragma omp parallel for schedule(static)
    for (std::ptrdiff_t i = 0; i < drone_count; ++i) {
        auto& drone = drones_[static_cast<size_t>(i)];
        drone.update(dt);
        if (use_entities) {
            drone.update_threat_and_fsm(target_entities_, dt);
        }
    }

    // Parallel sensor scans and combat engagements with thread-local buffers
    latest_observations_.clear();
    latest_damage_events_.clear();
    std::vector<std::vector<SensorObservation>> thread_observations;
    std::vector<std::vector<DamageEvent>> thread_damage_events;

    #pragma omp parallel
    {
        #pragma omp single
        {
            size_t num_threads = static_cast<size_t>(omp_get_num_threads());
            thread_observations.resize(num_threads);
            thread_damage_events.resize(num_threads);
        }

        int tid = omp_get_thread_num();
        const float radar_range = config.radar_range;
        const float inspection_range = config.inspection_range;
        const float firing_range = config.firing_range;

        #pragma omp for schedule(dynamic, 16)
        for (std::ptrdiff_t i = 0; i < drone_count; ++i) {
            auto& drone = drones_[static_cast<size_t>(i)];
            if (use_entities) {
                auto obs = drone.scan_entities(
                    target_entities_, current_step_, radar_range, inspection_range
                );
                if (!obs.empty()) {
                    auto& local_obs = thread_observations[static_cast<size_t>(tid)];
                    local_obs.insert(local_obs.end(), obs.begin(), obs.end());
                }
                drone.try_fire(
                    target_entities_,
                    current_step_,
                    firing_range,
                    thread_damage_events[static_cast<size_t>(tid)]
                );
            } else {
                auto obs = drone.scan_targets(targets_, current_step_, radar_range);
                if (!obs.empty()) {
                    auto& local_obs = thread_observations[static_cast<size_t>(tid)];
                    local_obs.insert(local_obs.end(), obs.begin(), obs.end());
                }
            }
        }
    }

    // Aggregate thread-local observations and damage events in deterministic thread order
    for (const auto& obs_vec : thread_observations) {
        latest_observations_.insert(latest_observations_.end(), obs_vec.begin(), obs_vec.end());
    }
    for (const auto& dmg_vec : thread_damage_events) {
        latest_damage_events_.insert(latest_damage_events_.end(), dmg_vec.begin(), dmg_vec.end());
    }

    // Apply local combat hits deterministically after parallel phase
    for (const auto& evt : latest_damage_events_) {
        apply_damage_event(evt);
    }
}

std::vector<DroneState> SimulationEngine::extract_boundary_crossing_drones() {
    std::vector<DroneState> migrating_out;
    if (num_ranks_ <= 1) {
        return migrating_out;
    }

    const float margin = get_global_config().migration_buffer_zone;

    for (auto& drone : drones_) {
        if (is_active_fsm_status(drone.get_status())) {
            Vector3 pos = drone.get_position();
            // Hysteresis check: avoid rapid ping-pong migrations near X = 25000 m
            bool crossed_out = false;
            if (rank_ == 0 && pos.x >= bounds_.max_x + margin) {
                crossed_out = true;
            } else if (rank_ > 0 && pos.x < bounds_.min_x - margin) {
                crossed_out = true;
            }

            if (crossed_out) {
                drone.set_status(DroneStatus::MIGRATING_OUT);
                migrating_out.push_back(drone.get_state());
            }
        }
    }

    return migrating_out;
}

bool SimulationEngine::ingest_migrated_drone(const DroneState& state) {
    DroneStatus restored_status = state.status;
    if (restored_status == DroneStatus::MIGRATING_OUT ||
        restored_status == DroneStatus::MIGRATING_IN ||
        restored_status == DroneStatus::IDLE) {
        restored_status = (state.assigned_task_id >= 0) ? DroneStatus::TASK_EXECUTING : DroneStatus::PATROL;
    }

    // De-duplication: check if drone is already locally present
    for (auto& drone : drones_) {
        if (drone.get_id() == state.id) {
            if (state.seq_num > drone.get_state().seq_num) {
                drone = Drone(state);
                drone.set_status(restored_status);
                drone.get_mutable_state().owner_rank = rank_;
            }
            return false;
        }
    }

    Drone new_drone(state);
    new_drone.set_status(restored_status);
    new_drone.get_mutable_state().owner_rank = rank_;
    drones_.push_back(new_drone);
    return true;
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

void SimulationEngine::rollback_migrated_out(int drone_id) {
    for (auto& drone : drones_) {
        if (drone.get_id() == drone_id && drone.get_status() == DroneStatus::MIGRATING_OUT) {
            DroneStatus restored =
                (drone.get_state().assigned_task_id >= 0) ? DroneStatus::TASK_EXECUTING : DroneStatus::PATROL;
            drone.set_status(restored);
            return;
        }
    }
}

std::vector<SensorObservation> SimulationEngine::collect_sensor_observations() {
    return latest_observations_;
}

std::vector<DamageEvent> SimulationEngine::collect_damage_events() const {
    return latest_damage_events_;
}

size_t SimulationEngine::get_active_drone_count() const {
    size_t active = 0;
    for (const auto& d : drones_) {
        if (is_active_fsm_status(d.get_status())) {
            active++;
        }
    }
    return active;
}

}  // namespace sim
