#include "comm/mpi_manager.hpp"
#include "core/config.hpp"
#include "fusion/sensor_fusion.hpp"
#include "migration/migration_manager.hpp"
#include "simulation/engine.hpp"
#include "task_allocation/task_coordinator.hpp"

#include <iostream>
#include <mpi.h>
#include <vector>

int main(int argc, char** argv) {
    sim::MpiManager mpi(&argc, &argv);
    const int rank = mpi.get_rank();
    const int size = mpi.get_size();
    const int peer_rank = mpi.get_peer_rank();

    if (size != 2) {
        if (rank == 0) {
            std::cerr << "[FAIL] phase3_test requires 2 MPI ranks (mpirun -np 2).\n";
        }
        return 1;
    }

    auto& config = sim::get_global_config();
    sim::SimulationEngine engine(rank, size);
    engine.initialize_agents(20);

    // Place a hostile target near the boundary in Region A (X = 24800 m)
    std::vector<sim::TargetEntity> entities = {
        sim::TargetEntity{101, sim::Vector3{24800.0f, 12000.0f, 100.0f}, 100.0f, true, 200.0f, true}
    };
    engine.set_target_entities(entities);

    auto& drones = engine.get_mutable_drones();
    if (rank == 0) {
        // Keep 18 drones in RECONNAISSANCE (busy), 1 free scout near target for inspection
        for (size_t i = 0; i < drones.size(); ++i) {
            auto& st = drones[i].get_mutable_state();
            if (i == 0) {
                st.position = sim::Vector3{24780.0f, 12000.0f, 100.0f};
                st.target_waypoint = st.position;
                st.status = sim::DroneStatus::PATROL;
            } else {
                st.position = sim::Vector3{5000.0f, 5000.0f, 100.0f};
                st.target_waypoint = st.position;
                st.status = sim::DroneStatus::RECONNAISSANCE;
            }
        }
    } else {
        // Rank 1 has free PATROL drones near X = 25150 m ready to reinforce across X = 25000 m
        for (size_t i = 0; i < drones.size(); ++i) {
            auto& st = drones[i].get_mutable_state();
            st.position = sim::Vector3{25150.0f, 12000.0f + static_cast<float>(i) * 10.0f, 100.0f};
            st.target_waypoint = st.position;
            st.status = sim::DroneStatus::PATROL;
        }
    }

    sim::MigrationManager migration_mgr(engine, mpi);
    sim::SensorFusionManager fusion_mgr(rank);
    sim::TaskCoordinator task_coord(rank, engine);

    if (rank == 0) {
        sim::Task inspect_task;
        inspect_task.task_id = 5001;
        inspect_task.type = sim::TaskType::OBJECT_INSPECTION;
        inspect_task.target_id = 101;
        inspect_task.location = entities[0].position;
        task_coord.submit_task(inspect_task);
    }

    mpi.barrier();

    bool target_eliminated = false;
    uint64_t total_combat_events = 0;

    for (int step = 1; step <= 250; ++step) {
        engine.step(config.delta_time);

        auto local_obs = engine.collect_sensor_observations();
        fusion_mgr.add_observations(local_obs);
        mpi.send_sensor_observations_batch(peer_rank, local_obs);

        auto local_dmg = engine.collect_damage_events();
        for (const auto& dmg : local_dmg) {
            total_combat_events++;
            if (dmg.hit) {
                fusion_mgr.apply_damage_to_track(dmg.target_id, dmg.damage);
                sim::CombatEventPayload evt{
                    dmg.target_id, dmg.attacker_drone_id, dmg.source_rank,
                    dmg.step, dmg.damage, dmg.hit
                };
                mpi.send_combat_event(peer_rank, evt);
            }
        }

        migration_mgr.process_outbound_migrations();

        std::vector<sim::MigrationPayload> in_mig;
        std::vector<sim::MigrationAckPayload> in_ack;
        std::vector<sim::HeartbeatPayload> in_hb;
        std::vector<sim::SensorObservation> in_obs;
        std::vector<sim::CombatEventPayload> in_combat;
        std::vector<sim::TaskAuctionPayload> in_ann;
        std::vector<sim::TaskBidPayload> in_bids;
        std::vector<sim::TaskClaimPayload> in_claims;

        mpi.poll_incoming_messages(
            in_mig, in_ack, in_hb, in_obs,
            &in_combat, &in_ann, &in_bids, &in_claims
        );

        if (!in_mig.empty()) migration_mgr.handle_incoming_migrations(in_mig);
        if (!in_ack.empty()) migration_mgr.handle_incoming_acks(in_ack);
        if (!in_obs.empty()) fusion_mgr.add_observations(in_obs);
        for (const auto& evt : in_combat) {
            if (evt.hit) {
                sim::DamageEvent dmg{
                    evt.target_id, evt.attacker_drone_id, evt.source_rank,
                    evt.step, evt.damage, evt.hit
                };
                engine.apply_damage_event(dmg);
                fusion_mgr.apply_damage_to_track(evt.target_id, evt.damage);
            }
        }

        task_coord.process_pending_tasks();
        task_coord.update_active_tasks(fusion_mgr);
        task_coord.evaluate_force_ratios(fusion_mgr);

        for (const auto& ann : task_coord.pop_outbound_announcements()) {
            mpi.send_task_announce(peer_rank, ann);
        }
        for (const auto& bid : task_coord.handle_incoming_announcements(in_ann)) {
            mpi.send_task_bid(bid.announcing_rank, bid);
        }
        for (const auto& claim : task_coord.handle_incoming_bids(in_bids)) {
            mpi.send_task_claim(claim.winning_rank, claim);
        }
        task_coord.handle_incoming_claims(in_claims);

        fusion_mgr.decay_tracks(static_cast<uint32_t>(step));
        migration_mgr.check_timeouts(static_cast<uint32_t>(step));

        if (!engine.get_target_entities()[0].alive) {
            target_eliminated = true;
        }
        mpi.barrier();
    }

    int local_elim = target_eliminated ? 1 : 0;
    int global_elim = 0;
    MPI_Allreduce(&local_elim, &global_elim, 1, MPI_INT, MPI_MIN, MPI_COMM_WORLD);

    uint64_t reinf = static_cast<uint64_t>(task_coord.get_reinforcements_dispatched());
    uint64_t total_reinf = 0;
    MPI_Allreduce(&reinf, &total_reinf, 1, MPI_UINT64_T, MPI_SUM, MPI_COMM_WORLD);

    uint64_t total_dmg_all = 0;
    MPI_Allreduce(&total_combat_events, &total_dmg_all, 1, MPI_UINT64_T, MPI_SUM, MPI_COMM_WORLD);

    bool passed = (global_elim == 1) && (total_reinf >= 1) && (total_dmg_all >= 2);

    if (rank == 0) {
        std::cout << "\n==============================================================\n";
        std::cout << " Phase 3 Fusion, Combat FSM & Cross-Sector Auction Test\n";
        std::cout << "==============================================================\n";
        std::cout << " Hostile Eliminated (Both Nodes) : " << (global_elim == 1 ? "YES" : "NO") << "\n";
        std::cout << " Cross-Sector Reinforcements     : " << total_reinf << "\n";
        std::cout << " Total Combat Shots Fired        : " << total_dmg_all << "\n";
        std::cout << " Status                          : " << (passed ? "PASSED" : "FAILED") << "\n";
        std::cout << "==============================================================\n";
    }

    return passed ? 0 : 1;
}
