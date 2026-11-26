#pragma once

#include "core/types.hpp"
#include <unordered_map>
#include <vector>

namespace sim {

struct FusedTargetTrack {
    int32_t target_id{0};
    Vector3 estimated_position;
    float cumulative_confidence{0.0f};
    float weight_sum{0.0f};
    uint32_t last_update_step{0};
    int32_t observation_count{0};
    bool classified{false};
    bool is_hostile{false};
    bool alive{true};
    float estimated_hp{100.0f};
};

class SensorFusionManager {
public:
    explicit SensorFusionManager(int rank);

    // Ingest local and remote sensor observations (with spatial clustering & time-weighted fusion)
    void add_observation(const SensorObservation& obs);

    // Add a batch of observations
    void add_observations(const std::vector<SensorObservation>& obs_list);

    // Decay track confidence over time using exponential decay
    void decay_tracks(uint32_t current_step);

    // Update target health/alive state from combat events
    void apply_damage_to_track(int32_t target_id, float damage);
    void mark_target_eliminated(int32_t target_id);

    // Query active fused tracks
    const std::unordered_map<int32_t, FusedTargetTrack>& get_tracks() const { return tracks_; }
    std::vector<FusedTargetTrack> get_hostile_tracks() const;
    std::vector<FusedTargetTrack> get_unclassified_tracks() const;
    size_t get_track_count() const { return tracks_.size(); }
    int get_rank() const { return rank_; }

private:
    int32_t find_cluster_track_id(const Vector3& pos, float cluster_radius) const;

    int rank_{0};
    int32_t next_synthetic_id_{-1};
    std::unordered_map<int32_t, FusedTargetTrack> tracks_;
};

}  // namespace sim
