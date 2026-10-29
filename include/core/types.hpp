#pragma once

#include <cmath>
#include <cstdint>
#include <string>

namespace sim {

// 3D Vector for kinematics and coordinates
struct Vector3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};

    constexpr Vector3() = default;
    constexpr Vector3(float px, float py, float pz) : x(px), y(py), z(pz) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3{x + other.x, y + other.y, z + other.z};
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3{x - other.x, y - other.y, z - other.z};
    }

    Vector3 operator*(float scalar) const {
        return Vector3{x * scalar, y * scalar, z * scalar};
    }

    float norm_sq() const { return x * x + y * y + z * z; }
    float norm() const { return std::sqrt(norm_sq()); }

    Vector3 normalized() const {
        float n = norm();
        if (n > 1e-6f) {
            return Vector3{x / n, y / n, z / n};
        }
        return Vector3{0.0f, 0.0f, 0.0f};
    }

    float distance_to(const Vector3& other) const {
        return (*this - other).norm();
    }
};

enum class DroneStatus : int32_t {
    IDLE = 0,
    PATROL = 1,
    MIGRATING_OUT = 2,
    MIGRATING_IN = 3,
    TASK_EXECUTING = 4,
    FAILED = 5
};

// Full serializable Drone State (used in migration & replication)
struct DroneState {
    int32_t id{0};
    int32_t owner_rank{0};
    Vector3 position;
    Vector3 velocity;
    Vector3 target_waypoint;
    float battery{100.0f};
    DroneStatus status{DroneStatus::IDLE};
    int32_t assigned_task_id{-1};
    uint32_t seq_num{0};
};

// Observation produced by an onboard drone sensor
struct SensorObservation {
    int32_t target_id{0};
    int32_t drone_id{0};
    int32_t source_rank{0};
    uint32_t step{0};
    Vector3 estimated_position;
    float confidence{1.0f};
};

// Task / Mission description
enum class TaskType : int32_t {
    RECONNAISSANCE = 0,
    OBJECT_INSPECTION = 1,
    AREA_SEARCH = 2
};

struct Task {
    int32_t task_id{0};
    TaskType type{TaskType::RECONNAISSANCE};
    Vector3 location;
    int32_t priority{1};
    int32_t assigned_drone_id{-1};
    int32_t assigned_rank{-1};
    bool completed{false};
};

// Spatial region boundaries
struct RegionBounds {
    float min_x{0.0f};
    float max_x{500.0f};
    float min_y{0.0f};
    float max_y{1000.0f};
    float min_z{0.0f};
    float max_z{200.0f};

    bool contains(const Vector3& pos) const {
        return pos.x >= min_x && pos.x < max_x &&
               pos.y >= min_y && pos.y <= max_y &&
               pos.z >= min_z && pos.z <= max_z;
    }
};

}  // namespace sim
