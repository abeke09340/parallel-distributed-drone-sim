#pragma once

#include "core/types.hpp"

namespace sim {

struct SimulationConfig {
    // World dimensions (25 km x 25 km per sector, 50 km x 25 km global)
    float world_width{50000.0f};   // X [0, 50000]
    float world_length{25000.0f};  // Y [0, 25000]
    float world_height{500.0f};    // Z [0, 500]

    // Spatial split boundary (Node 0 owns X < boundary, Node 1 owns X >= boundary)
    float partition_boundary_x{25000.0f};
    float migration_buffer_zone{250.0f};  // Hysteresis margin: [24750, 25250] m

    // Agent population & scale-matched kinematics/engagement
    int drones_per_node{500};
    float max_speed{90.0f};              // m/s (80-100 m/s for 25 km sector scale)
    float battery_drain_rate{0.02f};     // % per second
    float radar_range{1200.0f};          // m
    float inspection_range{35.0f};       // m (30-40 m close inspection)
    float firing_range{150.0f};          // m
    float hostile_threat_radius{200.0f}; // m

    // Timing
    float delta_time{0.05f};       // 20 Hz simulation step (50ms)
    int max_steps{2000};           // default benchmark step limit (0 = infinite)

    // OpenMP thread count (0 = auto-detect via omp_get_max_threads)
    int openmp_threads{0};

    // Fault tolerance
    int heartbeat_interval_steps{5};
    int heartbeat_timeout_steps{20};
    int checkpoint_interval_steps{50};

    // Helper: calculate bounds for a specific node rank
    RegionBounds get_region_bounds_for_rank(int rank, int num_ranks) const;
};

// Singleton / global accessor
SimulationConfig& get_global_config();

}  // namespace sim
