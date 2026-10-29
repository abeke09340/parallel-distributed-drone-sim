#include "simulation/drone.hpp"
#include "core/config.hpp"
#include <algorithm>

namespace sim {

Drone::Drone(const DroneState& state) : state_(state) {}

Drone::Drone(int id, int owner_rank, Vector3 initial_pos) {
    state_.id = id;
    state_.owner_rank = owner_rank;
    state_.position = initial_pos;
    state_.velocity = Vector3{0.0f, 0.0f, 0.0f};
    state_.target_waypoint = initial_pos;
    state_.battery = 100.0f;
    state_.status = DroneStatus::PATROL;
    state_.assigned_task_id = -1;
    state_.seq_num = 1;
}

void Drone::set_waypoint(const Vector3& wp) {
    state_.target_waypoint = wp;
}

void Drone::update(float dt) {
    if (state_.status == DroneStatus::FAILED || state_.status == DroneStatus::MIGRATING_OUT) {
        return;
    }

    const auto& config = get_global_config();

    // Direction vector towards waypoint
    Vector3 to_target = state_.target_waypoint - state_.position;
    float dist = to_target.norm();

    if (dist > 1.0f) {
        Vector3 dir = to_target.normalized();
        state_.velocity = dir * config.max_speed;
        state_.position = state_.position + (state_.velocity * dt);
    } else {
        state_.velocity = Vector3{0.0f, 0.0f, 0.0f};
    }

    // Battery discharge
    state_.battery = std::max(0.0f, state_.battery - (config.battery_drain_rate * dt));
    if (state_.battery <= 0.0f) {
        state_.status = DroneStatus::FAILED;
    }

    state_.seq_num++;
}

std::vector<SensorObservation> Drone::scan_targets(
    const std::vector<Vector3>& target_objects,
    uint32_t current_step,
    float max_range
) const {
    std::vector<SensorObservation> observations;
    if (state_.status == DroneStatus::FAILED || state_.status == DroneStatus::MIGRATING_OUT) {
        return observations;
    }

    for (size_t i = 0; i < target_objects.size(); ++i) {
        float dist = state_.position.distance_to(target_objects[i]);
        if (dist <= max_range) {
            SensorObservation obs;
            obs.target_id = static_cast<int32_t>(i);
            obs.drone_id = state_.id;
            obs.source_rank = state_.owner_rank;
            obs.step = current_step;
            obs.estimated_position = target_objects[i];
            obs.confidence = std::max(0.1f, 1.0f - (dist / max_range));
            observations.push_back(obs);
        }
    }

    return observations;
}

}  // namespace sim
