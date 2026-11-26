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
    state_.hp = 100.0f;
    state_.threat_level = 0.0f;
    state_.firing_target_id = -1;
    state_.fire_cooldown = 0.0f;
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
    } else if (state_.status == DroneStatus::PATROL) {
        // Waypoint reached in patrol: deterministically select a new patrol waypoint
        std::mt19937 rng(make_deterministic_seed(state_.id, state_.seq_num, 0xA5A5u));
        std::uniform_real_distribution<float> wp_x(20.0f, config.world_width - 20.0f);
        std::uniform_real_distribution<float> wp_y(20.0f, config.world_length - 20.0f);
        std::uniform_real_distribution<float> wp_z(10.0f, config.world_height - 10.0f);

        state_.target_waypoint = Vector3{wp_x(rng), wp_y(rng), wp_z(rng)};
        Vector3 dir = (state_.target_waypoint - state_.position).normalized();
        state_.velocity = dir * config.max_speed;
    } else {
        state_.velocity = Vector3{0.0f, 0.0f, 0.0f};
    }

    // Clamp position within global world boundaries
    state_.position.x = std::clamp(state_.position.x, 0.0f, config.world_width);
    state_.position.y = std::clamp(state_.position.y, 0.0f, config.world_length);
    state_.position.z = std::clamp(state_.position.z, 0.0f, config.world_height);

    // Battery discharge & weapon cooldown decay
    state_.battery = std::max(0.0f, state_.battery - (config.battery_drain_rate * dt));
    state_.fire_cooldown = std::max(0.0f, state_.fire_cooldown - dt);

    if (state_.battery <= 0.0f || state_.hp <= 0.0f) {
        state_.status = DroneStatus::FAILED;
        state_.velocity = Vector3{0.0f, 0.0f, 0.0f};
    }

    state_.seq_num++;
}

void Drone::update_threat_and_fsm(const std::vector<TargetEntity>& target_entities, float /*dt*/) {
    if (state_.status == DroneStatus::FAILED ||
        state_.status == DroneStatus::MIGRATING_OUT ||
        state_.status == DroneStatus::MIGRATING_IN) {
        return;
    }

    float max_proximity_threat = 0.0f;
    Vector3 nearest_hostile_pos{};
    bool has_nearby_hostile = false;

    for (const auto& target : target_entities) {
        if (!target.alive || !target.is_hostile) continue;
        float dist = state_.position.distance_to(target.position);
        float radius = std::max(1.0f, target.threat_radius);
        if (dist <= radius) {
            float t = std::clamp(1.0f - (dist / radius), 0.0f, 1.0f);
            if (t > max_proximity_threat) {
                max_proximity_threat = t;
                nearest_hostile_pos = target.position;
                has_nearby_hostile = true;
            }
        }
    }

    float health_penalty = (state_.hp < 40.0f) ? (40.0f - state_.hp) / 40.0f * 0.4f : 0.0f;
    float battery_penalty = (state_.battery < 20.0f) ? (20.0f - state_.battery) / 20.0f * 0.25f : 0.0f;
    state_.threat_level = std::clamp(max_proximity_threat + health_penalty + battery_penalty, 0.0f, 1.0f);

    // Automatic EVADE transition on critical threat or damaged combat drone
    if (has_nearby_hostile && (state_.threat_level >= 0.8f || (state_.hp < 40.0f && max_proximity_threat > 0.2f))) {
        state_.status = DroneStatus::EVADE;
        state_.assigned_task_id = -1;
        state_.firing_target_id = -1;

        Vector3 away = (state_.position - nearest_hostile_pos).normalized();
        if (away.norm_sq() < 1e-4f) {
            away = Vector3{1.0f, 0.0f, 0.0f};
        }
        const auto& config = get_global_config();
        state_.target_waypoint = Vector3{
            std::clamp(state_.position.x + away.x * 400.0f, 20.0f, config.world_width - 20.0f),
            std::clamp(state_.position.y + away.y * 400.0f, 20.0f, config.world_length - 20.0f),
            std::clamp(state_.position.z, 20.0f, config.world_height - 20.0f)
        };
    } else if (state_.status == DroneStatus::EVADE && state_.threat_level < 0.3f && state_.hp >= 40.0f) {
        // Recover to PATROL once clear of threat
        state_.status = DroneStatus::PATROL;
    }
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
            obs.is_hostile = false;
            obs.classified = false;
            observations.push_back(obs);
        }
    }

    return observations;
}

std::vector<SensorObservation> Drone::scan_entities(
    const std::vector<TargetEntity>& target_entities,
    uint32_t current_step,
    float max_range,
    float inspection_range
) const {
    std::vector<SensorObservation> observations;
    if (state_.status == DroneStatus::FAILED || state_.status == DroneStatus::MIGRATING_OUT) {
        return observations;
    }

    for (const auto& entity : target_entities) {
        if (!entity.alive) continue;

        float dist = state_.position.distance_to(entity.position);
        if (dist <= max_range) {
            uint32_t seed = make_deterministic_seed(
                state_.id, current_step, static_cast<uint32_t>(entity.id) + 1u
            );
            std::mt19937 rng(seed);
            float sigma = (dist <= inspection_range)
                              ? 0.2f
                              : (0.5f + 2.5f * (dist / max_range));
            std::normal_distribution<float> noise(0.0f, sigma);

            SensorObservation obs;
            obs.target_id = entity.id;
            obs.drone_id = state_.id;
            obs.source_rank = state_.owner_rank;
            obs.step = current_step;
            obs.estimated_position = Vector3{
                entity.position.x + noise(rng),
                entity.position.y + noise(rng),
                entity.position.z + noise(rng)
            };
            obs.confidence = std::max(0.1f, 1.0f - (dist / max_range));
            obs.classified = (dist <= inspection_range);
            obs.is_hostile = obs.classified ? entity.is_hostile : false;
            observations.push_back(obs);
        }
    }

    return observations;
}

void Drone::try_fire(
    const std::vector<TargetEntity>& target_entities,
    uint32_t current_step,
    float firing_range,
    std::vector<DamageEvent>& out_events
) {
    // Sole combat mode is TARGET_ELIMINATION with eligibility guard
    if (state_.status != DroneStatus::TARGET_ELIMINATION ||
        state_.hp < 40.0f ||
        state_.battery < 20.0f ||
        state_.fire_cooldown > 0.0f) {
        return;
    }

    const TargetEntity* chosen = nullptr;
    float best_dist = firing_range;

    for (const auto& target : target_entities) {
        if (!target.alive || !target.is_hostile) continue;
        if (state_.firing_target_id >= 0 && target.id != state_.firing_target_id) {
            continue;
        }
        float dist = state_.position.distance_to(target.position);
        if (dist <= best_dist) {
            best_dist = dist;
            chosen = &target;
        }
    }

    if (chosen == nullptr) {
        return;
    }

    // Deterministic 10% miss probability preserving OpenMP Thread-Invariance
    uint32_t seed = make_deterministic_seed(
        state_.id, current_step, static_cast<uint32_t>(chosen->id) + 777u
    );
    std::mt19937 rng(seed);
    bool is_hit = ((rng() % 10u) != 0u);  // 90% hit, 10% miss

    DamageEvent evt;
    evt.target_id = chosen->id;
    evt.attacker_drone_id = state_.id;
    evt.source_rank = state_.owner_rank;
    evt.step = current_step;
    evt.hit = is_hit;
    evt.damage = is_hit ? 50.0f : 0.0f;  // 50 HP/shot -> 2 hits eliminate 100 HP target
    out_events.push_back(evt);

    state_.fire_cooldown = 1.5f;  // 1.5 s weapon cooldown
}

}  // namespace sim
