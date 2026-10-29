#include "fusion/sensor_fusion.hpp"
#include <cmath>

namespace sim {

SensorFusionManager::SensorFusionManager(int rank) : rank_(rank) {}

void SensorFusionManager::add_observation(const SensorObservation& obs) {
    auto it = tracks_.find(obs.target_id);
    if (it == tracks_.end()) {
        // Initial track entry
        FusedTargetTrack track;
        track.target_id = obs.target_id;
        track.estimated_position = obs.estimated_position;
        track.cumulative_confidence = obs.confidence;
        track.last_update_step = obs.step;
        track.observation_count = 1;
        tracks_[obs.target_id] = track;
    } else {
        // Fused weighted average update
        auto& track = it->second;
        float old_weight = track.cumulative_confidence;
        float new_weight = obs.confidence;
        float total_weight = old_weight + new_weight;

        if (total_weight > 1e-4f) {
            track.estimated_position = (track.estimated_position * (old_weight / total_weight)) +
                                       (obs.estimated_position * (new_weight / total_weight));
            track.cumulative_confidence = std::min(1.0f, total_weight);
        }

        track.last_update_step = obs.step;
        track.observation_count++;
    }
}

void SensorFusionManager::add_observations(const std::vector<SensorObservation>& obs_list) {
    for (const auto& obs : obs_list) {
        add_observation(obs);
    }
}

void SensorFusionManager::decay_tracks(uint32_t current_step) {
    constexpr uint32_t MAX_AGE_STEPS = 100;

    for (auto it = tracks_.begin(); it != tracks_.end(); ) {
        uint32_t age = current_step - it->second.last_update_step;
        if (age > MAX_AGE_STEPS) {
            it = tracks_.erase(it);
        } else {
            // Exponential time decay
            it->second.cumulative_confidence *= 0.98f;
            ++it;
        }
    }
}

}  // namespace sim
