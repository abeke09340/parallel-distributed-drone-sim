#include "core/config.hpp"

namespace sim {

static SimulationConfig g_config;

SimulationConfig& get_global_config() {
    return g_config;
}

RegionBounds SimulationConfig::get_region_bounds_for_rank(int rank, int num_ranks) const {
    RegionBounds bounds;
    bounds.min_y = 0.0f;
    bounds.max_y = world_length;
    bounds.min_z = 0.0f;
    bounds.max_z = world_height;

    if (num_ranks <= 1) {
        bounds.min_x = 0.0f;
        bounds.max_x = world_width;
        return bounds;
    }

    // Default 2-node spatial partition
    if (rank == 0) {
        bounds.min_x = 0.0f;
        bounds.max_x = partition_boundary_x;
    } else {
        bounds.min_x = partition_boundary_x;
        bounds.max_x = world_width;
    }

    return bounds;
}

}  // namespace sim
