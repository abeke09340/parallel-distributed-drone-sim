#include "task_allocation/task_coordinator.hpp"
#include "core/config.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace sim {

TaskCoordinator::TaskCoordinator(int rank, SimulationEngine& engine)
    : rank_(rank), engine_(engine), next_task_id_(rank * 10000 + 100) {}

void TaskCoordinator::submit_task(const Task& task) {
    tasks_.push_back(task);
}

bool TaskCoordinator::is_drone_eligible(const Drone& drone, TaskType type) const {
    if (drone.get_status() != DroneStatus::PATROL) {
        return false;
    }
    const auto& s = drone.get_state();
    if (type == TaskType::TARGET_ELIMINATION) {
        // Combat eligibility guard per ROADMAP: hp >= 40, battery >= 20%
        return s.hp >= 40.0f && s.battery >= 20.0f;
    }
    return s.battery >= 10.0f;
}

float TaskCoordinator::evaluate_bid(const Drone& drone, const Task& task) const {
    if (!is_drone_eligible(drone, task.type)) {
        return -1.0f;
    }

    const auto& s = drone.get_state();
    float dist = s.position.distance_to(task.location);
    float dist_score = 1000.0f / std::max(50.0f, dist);
    float hp_factor = s.hp / 100.0f;
    float bat_factor = s.battery / 100.0f;
    float threat_penalty = 1.0f / (1.0f + s.threat_level);
    float workload_penalty = (s.assigned_task_id >= 0) ? 0.5f : 1.0f;

    switch (task.type) {
        case TaskType::TARGET_ELIMINATION:
            return dist_score * (0.5f * hp_factor + 0.3f * bat_factor + 0.2f) *
                   threat_penalty * workload_penalty;
        case TaskType::OBJECT_INSPECTION:
            return dist_score * (0.6f * bat_factor + 0.4f * hp_factor) *
                   threat_penalty * workload_penalty;
        case TaskType::RECONNAISSANCE:
        case TaskType::AREA_SEARCH:
        default:
            return dist_score * (0.7f * bat_factor + 0.3f * hp_factor) * workload_penalty;
    }
}

int TaskCoordinator::count_reserve_required() const {
    size_t active = engine_.get_active_drone_count();
    return std::max(1, static_cast<int>(std::ceil(static_cast<float>(active) * 0.20f)));
}

int TaskCoordinator::count_free_patrol_drones() const {
    int count = 0;
    for (const auto& d : engine_.get_drones()) {
        if (d.get_status() == DroneStatus::PATROL) {
            count++;
        }
    }
    return count;
}

bool TaskCoordinator::try_assign_local_drone(Task& task) {
    auto& drones = engine_.get_mutable_drones();

    int best_drone_idx = -1;
    float best_utility = -1.0f;

    for (size_t i = 0; i < drones.size(); ++i) {
        float utility = evaluate_bid(drones[i], task);
        if (utility > best_utility) {
            best_utility = utility;
            best_drone_idx = static_cast<int>(i);
        }
    }

    if (best_drone_idx < 0) {
        return false;
    }

    auto& chosen = drones[static_cast<size_t>(best_drone_idx)];
    DroneStatus new_status = DroneStatus::TASK_EXECUTING;
    if (task.type == TaskType::RECONNAISSANCE || task.type == TaskType::AREA_SEARCH) {
        new_status = DroneStatus::RECONNAISSANCE;
    } else if (task.type == TaskType::OBJECT_INSPECTION) {
        new_status = DroneStatus::OBJECT_INSPECTION;
    } else if (task.type == TaskType::TARGET_ELIMINATION) {
        new_status = DroneStatus::TARGET_ELIMINATION;
    }

    Vector3 initial_wp = (task.waypoint_count > 0) ? task.waypoints[0] : task.location;
    chosen.set_status(new_status);
    chosen.set_waypoint(initial_wp);
    chosen.get_mutable_state().assigned_task_id = task.task_id;
    if (task.type == TaskType::TARGET_ELIMINATION) {
        chosen.get_mutable_state().firing_target_id = task.target_id;
    }

    task.assigned_drone_id = chosen.get_id();
    task.assigned_rank = rank_;
    task.assigned_count++;

    std::cout << "[Task Allocation] Node " << rank_
              << " assigned Drone #" << task.assigned_drone_id
              << " to Task #" << task.task_id
              << " (Mode: " << static_cast<int>(task.type)
              << ", Utility: " << best_utility << ")\n";
    return true;
}

void TaskCoordinator::process_pending_tasks() {
    for (auto& task : tasks_) {
        if (!task.completed && task.assigned_drone_id == -1) {
            try_assign_local_drone(task);
        }
    }
}

void TaskCoordinator::update_active_tasks(const SensorFusionManager& fusion) {
    auto& drones = engine_.get_mutable_drones();
    const auto& tracks = fusion.get_tracks();
    const float inspection_range = get_global_config().inspection_range;

    for (auto& task : tasks_) {
        if (task.completed || task.assigned_drone_id < 0) continue;

        auto drone_it = std::find_if(drones.begin(), drones.end(),
            [&task](const Drone& d) { return d.get_id() == task.assigned_drone_id; });
        if (drone_it == drones.end()) {
            // Drone migrated out or no longer local; keep task tracked or completed
            continue;
        }
        if (drone_it->get_status() == DroneStatus::FAILED ||
            drone_it->get_status() == DroneStatus::EVADE) {
            task.assigned_drone_id = -1;
            continue;
        }

        if (task.type == TaskType::RECONNAISSANCE || task.type == TaskType::AREA_SEARCH) {
            if (task.waypoint_count > 0) {
                Vector3 wp = task.waypoints[task.current_waypoint_idx];
                if (drone_it->get_position().distance_to(wp) <= 30.0f) {
                    task.current_waypoint_idx++;
                    if (task.current_waypoint_idx >= task.waypoint_count) {
                        task.completed = true;
                    } else {
                        drone_it->set_waypoint(task.waypoints[task.current_waypoint_idx]);
                    }
                }
            } else if (drone_it->get_position().distance_to(task.location) <= 30.0f) {
                task.completed = true;
            }
        } else if (task.type == TaskType::OBJECT_INSPECTION) {
            auto tr_it = tracks.find(task.target_id);
            if (tr_it != tracks.end()) {
                drone_it->set_waypoint(tr_it->second.estimated_position);
                if (tr_it->second.classified ||
                    drone_it->get_position().distance_to(tr_it->second.estimated_position) <= inspection_range) {
                    task.completed = true;
                }
            }
        } else if (task.type == TaskType::TARGET_ELIMINATION) {
            auto tr_it = tracks.find(task.target_id);
            if (tr_it != tracks.end()) {
                if (!tr_it->second.alive) {
                    task.completed = true;
                } else {
                    task.location = tr_it->second.estimated_position;
                    drone_it->set_waypoint(task.location);
                }
            }
        }

        if (task.completed) {
            drone_it->set_status(DroneStatus::PATROL);
            drone_it->get_mutable_state().assigned_task_id = -1;
            drone_it->get_mutable_state().firing_target_id = -1;
        }
    }
}

void TaskCoordinator::evaluate_force_ratios(const SensorFusionManager& fusion) {
    const auto& bounds = engine_.get_bounds();
    auto hostile_tracks = fusion.get_hostile_tracks();

    for (const auto& track : hostile_tracks) {
        // Only the sector owning the hostile target initiates force-ratio coordination
        if (!bounds.contains(track.estimated_position)) {
            continue;
        }

        float friendly_power = 0.0f;
        for (const auto& d : engine_.get_drones()) {
            const auto& s = d.get_state();
            if (s.status == DroneStatus::TARGET_ELIMINATION &&
                s.firing_target_id == track.target_id &&
                s.hp >= 40.0f) {
                friendly_power += (s.hp / 100.0f);
            }
        }

        // RequiredPower = 1.5 * sum(HP_e / 100) accounting for 10% miss rate & return fire
        float required_power = 1.5f * (track.estimated_hp / 100.0f);
        int deficit = static_cast<int>(std::ceil(required_power - friendly_power));
        if (deficit <= 0) {
            continue;
        }

        int free_patrol = count_free_patrol_drones();
        int reserve = count_reserve_required();
        int pullable_local = std::max(0, free_patrol - reserve);
        int local_pull = std::min(deficit, pullable_local);

        for (int k = 0; k < local_pull; ++k) {
            Task combat_task;
            combat_task.task_id = next_task_id_++;
            combat_task.type = TaskType::TARGET_ELIMINATION;
            combat_task.location = track.estimated_position;
            combat_task.target_id = track.target_id;
            combat_task.priority = 10;
            combat_task.required_drones = 1;
            if (try_assign_local_drone(combat_task)) {
                tasks_.push_back(combat_task);
            } else {
                break;
            }
        }

        int remote_deficit = deficit - local_pull;
        if (remote_deficit > 0 &&
            open_remote_auctions_.find(track.target_id) == open_remote_auctions_.end()) {
            TaskAuctionPayload announce;
            announce.task_id = next_task_id_++;
            announce.type = TaskType::TARGET_ELIMINATION;
            announce.location = track.estimated_position;
            announce.target_id = track.target_id;
            announce.priority = 10;
            announce.announcing_rank = rank_;
            announce.remote_deficit = remote_deficit;

            open_remote_auctions_[track.target_id] = announce;
            outbound_announcements_.push_back(announce);
        }
    }
}

std::vector<TaskAuctionPayload> TaskCoordinator::pop_outbound_announcements() {
    auto result = std::move(outbound_announcements_);
    outbound_announcements_.clear();
    return result;
}

std::vector<TaskBidPayload> TaskCoordinator::handle_incoming_announcements(
    const std::vector<TaskAuctionPayload>& announcements
) {
    std::vector<TaskBidPayload> bids;
    const auto& drones = engine_.get_drones();
    int free_patrol = count_free_patrol_drones();
    int reserve = count_reserve_required();
    int available_for_reinforcement = std::max(0, free_patrol - reserve);

    if (available_for_reinforcement <= 0) {
        return bids;
    }

    for (const auto& ann : announcements) {
        Task temp_task;
        temp_task.task_id = ann.task_id;
        temp_task.type = ann.type;
        temp_task.location = ann.location;
        temp_task.target_id = ann.target_id;
        temp_task.priority = ann.priority;

        int best_id = -1;
        float best_util = -1.0f;
        for (const auto& d : drones) {
            float u = evaluate_bid(d, temp_task);
            if (u > best_util) {
                best_util = u;
                best_id = d.get_id();
            }
        }

        if (best_id >= 0) {
            TaskBidPayload bid;
            bid.task_id = ann.task_id;
            bid.target_id = ann.target_id;
            bid.candidate_drone_id = best_id;
            bid.bidder_rank = rank_;
            bid.announcing_rank = ann.announcing_rank;
            bid.utility = best_util;
            bids.push_back(bid);
        }
    }
    return bids;
}

std::vector<TaskClaimPayload> TaskCoordinator::handle_incoming_bids(
    const std::vector<TaskBidPayload>& bids
) {
    std::vector<TaskClaimPayload> claims;
    for (const auto& bid : bids) {
        auto it = open_remote_auctions_.find(bid.target_id);
        if (it == open_remote_auctions_.end() || it->second.remote_deficit <= 0) {
            continue;
        }

        TaskClaimPayload claim;
        claim.task_id = bid.task_id;
        claim.target_id = bid.target_id;
        claim.winning_drone_id = bid.candidate_drone_id;
        claim.winning_rank = bid.bidder_rank;
        claim.winning_bid = bid.utility;
        claim.location = it->second.location;
        claims.push_back(claim);

        it->second.remote_deficit--;
        if (it->second.remote_deficit <= 0) {
            open_remote_auctions_.erase(it);
        }
    }
    return claims;
}

void TaskCoordinator::handle_incoming_claims(const std::vector<TaskClaimPayload>& claims) {
    auto& drones = engine_.get_mutable_drones();
    for (const auto& claim : claims) {
        if (claim.winning_rank != rank_) continue;

        for (auto& d : drones) {
            if (d.get_id() == claim.winning_drone_id && d.get_status() == DroneStatus::PATROL) {
                d.set_status(DroneStatus::TARGET_ELIMINATION);
                d.set_waypoint(claim.location);
                d.get_mutable_state().assigned_task_id = claim.task_id;
                d.get_mutable_state().firing_target_id = claim.target_id;
                reinforcements_dispatched_++;

                std::cout << "[Reinforcement] Node " << rank_
                          << " dispatching Drone #" << d.get_id()
                          << " across boundary to assist Target #" << claim.target_id << "\n";
                break;
            }
        }
    }
}

size_t TaskCoordinator::get_completed_task_count() const {
    size_t count = 0;
    for (const auto& t : tasks_) {
        if (t.completed) count++;
    }
    return count;
}

}  // namespace sim
