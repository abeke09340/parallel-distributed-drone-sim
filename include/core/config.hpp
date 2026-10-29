#pragma once

#include "core/types.hpp"

namespace sim {

struct SimulationConfig {
    // World dimensions
    float world_width{1000.0f};   // X [0, 1000]
    float world_length{1000.0f};  // Y [0, 1000]
    float world_height{200.0f};   // Z [0, 200]

    // Spatial split boundary (Node 0 owns X < boundary, Node 1 owns X >= boundary)
    float partition_boundary_x{500.0f};
    float migration_buffer_zone{15.0f};

    // Agent population
    int drones_per_node{500};
    float max_speed{15.0f};        // m/s
    float battery_drain_rate{0.02f}; // % per second

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
