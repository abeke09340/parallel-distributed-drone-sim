#pragma once

#include "comm/protocol.hpp"
#include "core/types.hpp"
#include "fusion/sensor_fusion.hpp"
#include "simulation/engine.hpp"
#include <unordered_map>
#include <vector>

namespace sim {

class TaskCoordinator {
public:
    TaskCoordinator(int rank, SimulationEngine& engine);

    // Register a new pending task
    void submit_task(const Task& task);

    // Evaluate mode-weighted utility U = f(dist, HP, battery, threat, workload)
    float evaluate_bid(const Drone& drone, const Task& task) const;

    // Evaluate local bids and assign the best available local drone
    bool try_assign_local_drone(Task& task);

    // Assign pending tasks and advance multi-waypoint / inspection / elimination progress
    void process_pending_tasks();
    void update_active_tasks(const SensorFusionManager& fusion);

    // Dynamic Force-Ratio Analysis: pull local drones (keeping 20% reserve) & emit remote_deficit
    void evaluate_force_ratios(const SensorFusionManager& fusion);

    // Distributed MPI Consensus handlers (MSG_TASK_ANNOUNCE, MSG_TASK_BID, MSG_TASK_CLAIM)
    std::vector<TaskAuctionPayload> pop_outbound_announcements();
    std::vector<TaskBidPayload> handle_incoming_announcements(
        const std::vector<TaskAuctionPayload>& announcements
    );
    std::vector<TaskClaimPayload> handle_incoming_bids(
        const std::vector<TaskBidPayload>& bids
    );
    void handle_incoming_claims(const std::vector<TaskClaimPayload>& claims);

    const std::vector<Task>& get_tasks() const { return tasks_; }
    size_t get_completed_task_count() const;
    size_t get_reinforcements_dispatched() const { return reinforcements_dispatched_; }

private:
    bool is_drone_eligible(const Drone& drone, TaskType type) const;
    int count_reserve_required() const;
    int count_free_patrol_drones() const;

    int rank_{0};
    SimulationEngine& engine_;
    std::vector<Task> tasks_;
    int32_t next_task_id_{1000};
    size_t reinforcements_dispatched_{0};

    std::vector<TaskAuctionPayload> outbound_announcements_;
    std::unordered_map<int32_t, TaskAuctionPayload> open_remote_auctions_;
};

}  // namespace sim
