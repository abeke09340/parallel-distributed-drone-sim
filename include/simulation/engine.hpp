#pragma once

#include "core/types.hpp"
#include "simulation/drone.hpp"
#include <vector>
#include <memory>

namespace sim {

class SimulationEngine {
public:
    SimulationEngine(int rank, int num_ranks);

    // Initialize initial swarm of drones inside local region
    void initialize_agents(int count);

    // Set simulated targets in the world
    void set_targets(const std::vector<Vector3>& targets);

    // Execute one simulation step (parallelized with OpenMP)
    void step(float dt);

    // Extract drones that crossed into foreign territory
    std::vector<DroneState> extract_boundary_crossing_drones();

    // Ingest drone migrating into this node
    void ingest_migrated_drone(const DroneState& state);

    // Complete migration out (clean up acknowledged drone)
    void finalize_migrated_out(int drone_id);

    // Collect all local sensor observations made this step
    std::vector<SensorObservation> collect_sensor_observations();

    // Accessors
    const std::vector<Drone>& get_drones() const { return drones_; }
    std::vector<Drone>& get_mutable_drones() { return drones_; }
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
    std::vector<SensorObservation> latest_observations_;
};

}  // namespace sim
