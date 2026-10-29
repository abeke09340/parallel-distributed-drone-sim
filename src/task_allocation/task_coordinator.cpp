#include "task_allocation/task_coordinator.hpp"
#include <limits>
#include <iostream>

namespace sim {

TaskCoordinator::TaskCoordinator(int rank, SimulationEngine& engine)
    : rank_(rank), engine_(engine) {}

void TaskCoordinator::submit_task(const Task& task) {
    tasks_.push_back(task);
}

bool TaskCoordinator::try_assign_local_drone(Task& task) {
    auto& drones = engine_.get_mutable_drones();

    int best_drone_idx = -1;
    float best_utility = -1.0f;

    for (size_t i = 0; i < drones.size(); ++i) {
        if (drones[i].get_status() == DroneStatus::PATROL) {
            float dist = drones[i].get_position().distance_to(task.location);
            float battery = drones[i].get_state().battery;

            // Simple utility model: high battery, low distance
            float utility = (battery * 0.5f) / std::max(1.0f, dist);
            if (utility > best_utility) {
                best_utility = utility;
                best_drone_idx = static_cast<int>(i);
            }
        }
    }

    if (best_drone_idx >= 0) {
        drones[best_drone_idx].set_status(DroneStatus::TASK_EXECUTING);
        drones[best_drone_idx].set_waypoint(task.location);
        drones[best_drone_idx].get_mutable_state().assigned_task_id = task.task_id;

        task.assigned_drone_id = drones[best_drone_idx].get_id();
        task.assigned_rank = rank_;

        std::cout << "[Task Allocation] Node " << rank_
                  << " assigned Drone #" << task.assigned_drone_id
                  << " to Task #" << task.task_id << " (Utility: " << best_utility << ")\n";
        return true;
    }

    return false;
}

void TaskCoordinator::process_pending_tasks() {
    for (auto& task : tasks_) {
        if (!task.completed && task.assigned_drone_id == -1) {
            try_assign_local_drone(task);
        }
    }
}

}  // namespace sim
