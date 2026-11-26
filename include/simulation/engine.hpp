#pragma once

#include "core/types.hpp"
#include "simulation/drone.hpp"
#include <memory>
#include <vector>

namespace sim {

class SimulationEngine {
public:
    SimulationEngine(int rank, int num_ranks);

    // Initialize initial swarm of drones inside local region
    void initialize_agents(int count);

    // Set simulated targets (raw positions for Phase 1 compatibility or full TargetEntity)
    void set_targets(const std::vector<Vector3>& targets);
    void set_target_entities(const std::vector<TargetEntity>& entities);

    // Execute one simulation step (parallelized with OpenMP)
    void step(float dt);

    // Apply local or remote combat damage event to target entities
    void apply_damage_event(const DamageEvent& evt);

    // Extract drones that crossed into foreign territory (with hysteresis)
    std::vector<DroneState> extract_boundary_crossing_drones();

    // Ingest drone migrating into this node (returns true if newly added, false if duplicate)
    bool ingest_migrated_drone(const DroneState& state);

    // Complete migration out (clean up acknowledged drone)
    void finalize_migrated_out(int drone_id);

    // Roll back timed-out or rejected migration (restore drone to active status)
    void rollback_migrated_out(int drone_id);

    // Collect all local sensor observations and combat events made this step
    std::vector<SensorObservation> collect_sensor_observations();
    std::vector<DamageEvent> collect_damage_events() const;

    // Accessors
    const std::vector<Drone>& get_drones() const { return drones_; }
    std::vector<Drone>& get_mutable_drones() { return drones_; }
    const std::vector<TargetEntity>& get_target_entities() const { return target_entities_; }
    std::vector<TargetEntity>& get_mutable_target_entities() { return target_entities_; }
    const RegionBounds& get_bounds() const { return bounds_; }
    int get_rank() const { return rank_; }
    int get_num_ranks() const { return num_ranks_; }
    uint32_t get_current_step() const { return current_step_; }
    size_t get_active_drone_count() const;

private:
    int rank_{0};
    int num_ranks_{1};
    RegionBounds bounds_;
    uint32_t current_step_{0};

    std::vector<Drone> drones_;
    std::vector<Vector3> targets_;
    std::vector<TargetEntity> target_entities_;
    std::vector<SensorObservation> latest_observations_;
    std::vector<DamageEvent> latest_damage_events_;
};

}  // namespace sim
