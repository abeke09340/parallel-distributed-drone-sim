#include "fusion/sensor_fusion.hpp"
#include <algorithm>
#include <cmath>

namespace sim {

namespace {
constexpr float TIME_DECAY_LAMBDA = 0.04f;
constexpr float CLUSTER_RADIUS_METERS = 150.0f;
}  // namespace

SensorFusionManager::SensorFusionManager(int rank) : rank_(rank) {}

int32_t SensorFusionManager::find_cluster_track_id(const Vector3& pos, float cluster_radius) const {
    int32_t best_id = 0;
    float best_dist = cluster_radius;
    bool found = false;

    for (const auto& [id, track] : tracks_) {
        if (!track.alive) continue;
        float dist = track.estimated_position.distance_to(pos);
        if (dist <= best_dist) {
            best_dist = dist;
            best_id = id;
            found = true;
        }
    }
    return found ? best_id : 0;
}

void SensorFusionManager::add_observation(const SensorObservation& obs) {
    int32_t key = obs.target_id;
    auto it = tracks_.find(key);

    // Spatial clustering fallback for unassociated observations (target_id < 0)
    if (it == tracks_.end() && obs.target_id < 0) {
        int32_t clustered_id = find_cluster_track_id(obs.estimated_position, CLUSTER_RADIUS_METERS);
        if (clustered_id != 0) {
            key = clustered_id;
            it = tracks_.find(key);
        } else {
            key = next_synthetic_id_--;
        }
    }

    if (it == tracks_.end()) {
        FusedTargetTrack track;
        track.target_id = key;
        track.estimated_position = obs.estimated_position;
        track.cumulative_confidence = std::clamp(obs.confidence, 0.05f, 1.0f);
        track.weight_sum = track.cumulative_confidence;
        track.last_update_step = obs.step;
        track.observation_count = 1;
        track.classified = obs.classified;
        track.is_hostile = obs.is_hostile;
        track.alive = true;
        track.estimated_hp = 100.0f;
        tracks_[key] = track;
    } else {
        auto& track = it->second;
        if (!track.alive) {
            return;
        }

        // Exponential time-decay on accumulated weight: w_old * exp(-lambda * dt_steps)
        uint32_t dt_steps = (obs.step > track.last_update_step)
                                ? (obs.step - track.last_update_step)
                                : 0;
        float decayed_old_weight = track.weight_sum * std::exp(-TIME_DECAY_LAMBDA * static_cast<float>(dt_steps));
        float new_weight = std::clamp(obs.confidence, 0.05f, 1.0f);
        float total_weight = decayed_old_weight + new_weight;

        if (total_weight > 1e-5f) {
            // Confidence- and time-weighted position fusion: P_hat = sum(w_i * P_i) / sum(w_i)
            track.estimated_position =
                (track.estimated_position * (decayed_old_weight / total_weight)) +
                (obs.estimated_position * (new_weight / total_weight));
            track.weight_sum = std::min(5.0f, total_weight);
            track.cumulative_confidence = std::min(1.0f, total_weight);
        }

        if (obs.classified) {
            track.classified = true;
            track.is_hostile = obs.is_hostile;
        }

        if (obs.step >= track.last_update_step) {
            track.last_update_step = obs.step;
        }
        track.observation_count++;
    }
}

void SensorFusionManager::add_observations(const std::vector<SensorObservation>& obs_list) {
    for (const auto& obs : obs_list) {
        add_observation(obs);
    }
}

void SensorFusionManager::decay_tracks(uint32_t current_step) {
    constexpr uint32_t MAX_AGE_STEPS = 120;

    for (auto it = tracks_.begin(); it != tracks_.end();) {
        uint32_t age = (current_step > it->second.last_update_step)
                           ? (current_step - it->second.last_update_step)
                           : 0;
        if (age > MAX_AGE_STEPS || !it->second.alive) {
            it = tracks_.erase(it);
        } else {
            float decay = std::exp(-TIME_DECAY_LAMBDA);
            it->second.cumulative_confidence *= decay;
            it->second.weight_sum *= decay;
            ++it;
        }
    }
}

void SensorFusionManager::apply_damage_to_track(int32_t target_id, float damage) {
    auto it = tracks_.find(target_id);
    if (it == tracks_.end()) return;
    it->second.estimated_hp = std::max(0.0f, it->second.estimated_hp - damage);
    if (it->second.estimated_hp <= 0.0f) {
        it->second.alive = false;
    }
}

void SensorFusionManager::mark_target_eliminated(int32_t target_id) {
    auto it = tracks_.find(target_id);
    if (it == tracks_.end()) return;
    it->second.estimated_hp = 0.0f;
    it->second.alive = false;
}

std::vector<FusedTargetTrack> SensorFusionManager::get_hostile_tracks() const {
    std::vector<FusedTargetTrack> result;
    for (const auto& [id, track] : tracks_) {
        if (track.alive && track.classified && track.is_hostile) {
            result.push_back(track);
        }
    }
    return result;
}

std::vector<FusedTargetTrack> SensorFusionManager::get_unclassified_tracks() const {
    std::vector<FusedTargetTrack> result;
    for (const auto& [id, track] : tracks_) {
        if (track.alive && !track.classified) {
            result.push_back(track);
        }
    }
    return result;
}

}  // namespace sim
