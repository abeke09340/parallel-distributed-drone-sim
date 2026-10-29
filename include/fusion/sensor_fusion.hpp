#pragma once

#include "core/types.hpp"
#include <unordered_map>
#include <vector>

namespace sim {

struct FusedTargetTrack {
    int32_t target_id{0};
    Vector3 estimated_position;
    float cumulative_confidence{0.0f};
    uint32_t last_update_step{0};
    int32_t observation_count{0};
};

class SensorFusionManager {
public:
    SensorFusionManager(int rank);

    // Ingest local and remote sensor observations
    void add_observation(const SensorObservation& obs);

    // Add a batch of observations
    void add_observations(const std::vector<SensorObservation>& obs_list);

    // Decay track confidence over time
    void decay_tracks(uint32_t current_step);

    // Access active fused tracks
    const std::unordered_map<int32_t, FusedTargetTrack>& get_tracks() const { return tracks_; }
    size_t get_track_count() const { return tracks_.size(); }
    int get_rank() const { return rank_; }

private:
    int rank_{0};
    std::unordered_map<int32_t, FusedTargetTrack> tracks_;
};

}  // namespace sim
