#include "simulation/drone.hpp"
#include "core/config.hpp"
#include <algorithm>
#include <random>

namespace sim {

namespace {
// Deterministic hash-based seed generator for thread-safe parallel execution.
// Produces identical results regardless of OpenMP thread count or scheduling order.
inline uint32_t make_deterministic_seed(int32_t drone_id, uint32_t seq_or_step, uint32_t salt) {
    uint32_t h = static_cast<uint32_t>(drone_id) * 2654435761u;
    h ^= seq_or_step * 2246822519u;
    h ^= salt * 3266489917u;
    h ^= (h >> 16);
    h *= 0x85ebca6bu;
    h ^= (h >> 13);
    return h;
}
}  // namespace

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
        // Waypoint reached: deterministically select a new patrol waypoint
        std::mt19937 rng(make_deterministic_seed(state_.id, state_.seq_num, 0xA5A5u));
        std::uniform_real_distribution<float> wp_x(20.0f, config.world_width - 20.0f);
        std::uniform_real_distribution<float> wp_y(20.0f, config.world_length - 20.0f);
        std::uniform_real_distribution<float> wp_z(10.0f, config.world_height - 10.0f);

        state_.target_waypoint = Vector3{wp_x(rng), wp_y(rng), wp_z(rng)};
        Vector3 dir = (state_.target_waypoint - state_.position).normalized();
        state_.velocity = dir * config.max_speed;
    }

    // Clamp position within global world boundaries
    state_.position.x = std::clamp(state_.position.x, 0.0f, config.world_width);
    state_.position.y = std::clamp(state_.position.y, 0.0f, config.world_length);
    state_.position.z = std::clamp(state_.position.z, 0.0f, config.world_height);

    // Battery discharge
    state_.battery = std::max(0.0f, state_.battery - (config.battery_drain_rate * dt));
    if (state_.battery <= 0.0f) {
        state_.status = DroneStatus::FAILED;
        state_.velocity = Vector3{0.0f, 0.0f, 0.0f};
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
            // Distance-proportional Gaussian sensor noise (deterministic per drone, target, and step)
            uint32_t seed = make_deterministic_seed(
                state_.id, current_step, static_cast<uint32_t>(i) + 1u
            );
            std::mt19937 rng(seed);
            float sigma = 0.5f + 2.5f * (dist / max_range);
            std::normal_distribution<float> noise(0.0f, sigma);

            Vector3 noisy_pos{
                target_objects[i].x + noise(rng),
                target_objects[i].y + noise(rng),
                target_objects[i].z + noise(rng)
            };

            SensorObservation obs;
            obs.target_id = static_cast<int32_t>(i);
            obs.drone_id = state_.id;
            obs.source_rank = state_.owner_rank;
            obs.step = current_step;
            obs.estimated_position = noisy_pos;
            obs.confidence = std::max(0.1f, 1.0f - (dist / max_range));
            observations.push_back(obs);
        }
    }

    return observations;
}

}  // namespace sim
