#pragma once

#include "core/types.hpp"
#include <vector>

namespace sim {

class Drone {
public:
    Drone() = default;
    explicit Drone(const DroneState& state);
    Drone(int id, int owner_rank, Vector3 initial_pos);

    // Update kinematic physics and battery for one tick dt
    void update(float dt);

    // Scan for targets within sensor range
    std::vector<SensorObservation> scan_targets(
        const std::vector<Vector3>& target_objects,
        uint32_t current_step,
        float max_range
    ) const;

    // Set steering target waypoint
    void set_waypoint(const Vector3& wp);

    // Get current state snapshot
    const DroneState& get_state() const { return state_; }
    DroneState& get_mutable_state() { return state_; }

    int get_id() const { return state_.id; }
    Vector3 get_position() const { return state_.position; }
    DroneStatus get_status() const { return state_.status; }

    void set_status(DroneStatus status) { state_.status = status; }

private:
    DroneState state_;
};

}  // namespace sim
