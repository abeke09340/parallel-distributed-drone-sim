#pragma once

#include "core/types.hpp"
#include "simulation/engine.hpp"
#include <vector>

namespace sim {

class TaskCoordinator {
public:
    TaskCoordinator(int rank, SimulationEngine& engine);

    // Register a new pending task
    void submit_task(const Task& task);

    // Evaluate local bids and assign the best available local drone
    bool try_assign_local_drone(Task& task);

    // Assign tasks for this step
    void process_pending_tasks();

    const std::vector<Task>& get_tasks() const { return tasks_; }

private:
    int rank_{0};
    SimulationEngine& engine_;
    std::vector<Task> tasks_;
};

}  // namespace sim
