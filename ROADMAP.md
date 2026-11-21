# Project Roadmap

This roadmap breaks down the development of the **Distributed Multi-Agent Simulation** into 4 structured milestones. Work proceeds collectively as a single unified team.

---

## 📅 Milestones Overview

```
Phase 1: Foundation & Local Parallelism (Week 1 - 2)
   └── Spatial partitioning, OpenMP drone simulation, headless CLI runner

Phase 2: Distributed Messaging & Agent Migration (Week 3 - 4)
   └── MPI setup, 2-phase migration protocol, non-blocking comms

Phase 3: Sensor Fusion & Task Allocation (Week 5 - 6)
   └── Decentralized track fusion, auction-based consensus, target generator

Phase 4: Fault Tolerance, Evaluation & Visualization (Week 7 - 8)
   └── Heartbeats, node recovery, network partition tests, Unity bridge
```

---

## Phase 1: Foundation & Local Parallel Engine
**Objective:** Single-node multi-drone physics and OpenMP benchmark.

- [x] **Project Setup & CMake scaffolding** (`CMakeLists.txt`, headers, git setup, optional MPI support)
- [x] **Drone agent data structures & kinematics** (`types.hpp`, `drone.cpp`)
  - Position, velocity, battery drain, continuous waypoint steering & boundary clamping.
- [x] **Spatial Region & OpenMP Engine** (`engine.cpp`)
  - Parallel loop (`#pragma omp parallel for`) for 500+ drones with deterministic thread-invariance.
- [x] **Sensory Field-of-View & Target generation** (`drone.cpp`, `sensor_fusion.hpp`)
  - Distance checks, Gaussian noise simulation, observation records.
- [x] **Basic CLI telemetry & benchmarks** (`src/benchmark/local_benchmark.cpp`)
  - Measuring tick execution time, speedup, and efficiency vs OpenMP thread count (1, 2, 4, 8 threads).

---

## Phase 2: Distributed MPI & Agent Migration
**Objective:** Two nodes exchanging drone state dynamically when crossing the boundary (`X = 25000.0 m`).

- [x] **World Scale & Spatial Geometry Update** (`config.hpp`, `types.hpp`)
  - Scale each sector to `25 km × 25 km` (Global world `50 km × 25 km`, Node 0: `X ∈ [0, 25000)`, Node 1: `X ∈ [25000, 50000]`, buffer zone `±250 m`, 500 drones/sector = 2 drones per 100 m of sector side).
  - Add flat POD fields to `DroneState` (`hp`, `threat_level`, `firing_target_id`, `fire_cooldown`) and `TargetEntity` (`hp`, `is_hostile`, `threat_radius`, `alive`).
- [x] **MPI Lifecycle Manager** (`mpi_manager.cpp`)
  - Initialize MPI, rank identification (Rank 0 = Region A, Rank 1 = Region B), non-blocking `MPI_Isend` / batching.
- [x] **Message Protocol & Binary Serialization** (`protocol.hpp`)
  - Packed POD structs for MPI send/recv, including `TAG_COMBAT_EVENT` for cross-node hit synchronization.
- [x] **2-Phase Migration Protocol** (`migration_manager.cpp`)
  - Detect `X >= 25000.0` (with `±250 m` hysteresis), serialize drone (preserving `hp`, `threat_level`, active mission), send `MSG_MIGRATION_REQ`, wait for `MSG_MIGRATION_ACK` + timeout/rollback.
- [x] **Target Node Ingestion & De-duplication**
  - Receive drone payload on Rank 1, instantiate local drone, send ACK.
- [x] **Migration Latency & Zero-Duplicate Verification Tests** (`tests/migration_test.cpp`)
  - Verified with `mpirun -np 2`: 1000 drones conserved (`Rank 0` + `Rank 1` == 1000 unique IDs), bidirectional migration, non-blocking `MPI_Isend`, timeout rollback + retry recovery.

---

### 📝 Phase 2 Implementation Notes (for Team / AI Context)
> **Summary of additions beyond base bullet points:**
> - `include/core/types.hpp`: Added `id` field to `TargetEntity` and `static_assert(std::is_trivially_copyable_v<...>)` for `DroneState` and `TargetEntity`.
> - `include/comm/protocol.hpp`: Added `CombatEventPayload` struct for `TAG_COMBAT_EVENT` and `static_assert(std::is_trivially_copyable_v<...>)` for all MPI payloads.
> - `include/comm/mpi_manager.hpp` & `src/comm/mpi_manager.cpp`: Added `MPI_Isend` buffer management (`pending_sends_`, `flush_completed_sends()`), batch APIs (`send_migration_batch()`, `send_sensor_observations_batch()`), `send_combat_event()`, and optional 5th parameter `incoming_combat_events = nullptr` in `poll_incoming_messages()` to preserve `main.cpp` compatibility.
> - `include/simulation/engine.hpp` & `src/simulation/engine.cpp`: `ingest_migrated_drone()` returns `bool` (true if new, false if duplicate) and preserves `TASK_EXECUTING` status; added `rollback_migrated_out()`.
> - `include/migration/migration_manager.hpp` & `src/migration/migration_manager.cpp`: Added `check_timeouts(current_step, timeout_steps)`, `seq_num` ACK validation, rejection/timeout rollback, and `get_total_rollbacks()` / `get_pending_count()`.
> - `tests/migration_test.cpp`: New 2-rank MPI test verifying zero duplicates, zero losses, and migration poll latency.

---

## Phase 3: Sensor Fusion, Combat/Threat Logic & Distributed Task Allocation
**Objective:** Multi-node perception, basic threat/combat FSM, and decentralized 3-mode mission planning.

- [ ] **Decentralized Track Manager** (`sensor_fusion.cpp`)
  - Multi-observation clustering for hostile and non-hostile targets (radar range `1200 m`).
- [ ] **Time- & Confidence-Weighted Fusion**
  - Equation: $\hat{P} = \sum w_i P_i / \sum w_i$ with exponential time decay.
- [ ] **Basic Threat Assessment, Combat & Drone FSM** (`drone.cpp`, `engine.cpp`)
  - Equal drone damage (`50 HP` / shot, 2 hits = target elimination, `fire_cooldown = 1.5 s`) with deterministic **10% miss probability** (`make_deterministic_seed` — preserves Thread-Invariance), thread-local `DamageEvent` buffers in OpenMP, `TAG_COMBAT_EVENT` sync over MPI.
  - Basic `threat_level ∈ [0, 1]` (hostile proximity + low HP/battery) and automatic mode switching (`PATROL` ↔ `RECONNAISSANCE` ↔ `OBJECT_INSPECTION` ↔ `TARGET_ELIMINATION` ↔ `EVADE` / `FAILED`).
- [ ] **Distributed 3-Mode Task Auction & Cross-Sector Reinforcements** (`task_coordinator.cpp`)
  - **3 Task Modes:**
    1. `RECONNAISSANCE` — POD `waypoints[3]` (`waypoint_count = 1` for point check, `2..3` for area search; completes when all waypoints visited or object detected).
    2. `OBJECT_INSPECTION` — close approach (`30–40 m`) to minimize sensor noise and classify target (`is_hostile` vs non-hostile).
    3. `TARGET_ELIMINATION` — sole combat mode with `TAG_COMBAT_EVENT` sync (eligibility guard: `hp >= 40`, `battery >= 20%`).
  - **Dynamic Force-Ratio Analysis & Smart Reinforcements:** for any detected hostile group ($N \ge 1$, even 1–2 enemies), compare local effective combat power ($\text{FriendlyPower} = \sum \text{HP}_d / 100$ for drones with $\text{HP} \ge 40$) against required force with safety margin ($\text{RequiredPower} = 1.5 \cdot \sum \text{HP}_e / 100$, accounting for 10% miss rate & return fire). Whenever $\text{deficit} = \lceil \text{RequiredPower} - \text{FriendlyPower} \rceil > 0$, pull free drones locally first (keeping a 20% sector reserve) and broadcast any remaining `remote_deficit` via `MSG_TASK_ANNOUNCE` over MPI so reinforcements migrate across `X = 25000 m`.
  - Mode-weighted bid evaluation: $U = f(\text{dist}, \text{HP}, \text{battery}, \text{threat}, \text{workload})$.
- [ ] **Consensus Resolution over MPI** (`MSG_TASK_ANNOUNCE`, `MSG_TASK_BID`, `MSG_TASK_CLAIM`)

---

## Phase 4: Fault Tolerance, Evaluation & Visualization
**Objective:** Node failure recovery, network partition tolerance, multi-zoom visualization, and final evaluation.

- [ ] **Heartbeat Monitor & Timeout Handler** (`fault_manager.cpp`)
  - Periodic ping/pong, detect missing rank after threshold, switch to autonomous mode (freeze border crossing).
- [ ] **State Checkpointing & Asynchronous Replication**
  - Periodic snapshot of active mission tracks, drone HP/states, alive/eliminated targets, and border states (`TAG_CHECKPOINT_SYNC`).
- [ ] **Split-Brain & Partition Reconciliation**
  - Simulate network cut (`mpirun` partition injection), resume from checkpoint and reconcile sequence logs.
- [ ] **Telemetry Bridge & Multi-Zoom Visualizer** (`bridge.cpp`, `visualization/`)
  - Decoupled non-blocking TCP/WebSocket server broadcasting throttled (10–20 FPS) NaN-safe JSON snapshots to HTML5 Canvas / Unity.
  - **6 Non-Linear Zoom Levels with LOD & Viewport Culling:**
    - `25%` (`50 km` / full `25 km` sector) — points & border only (no labels/HP bars)
    - `50%` (`10 km`) — swarm groups, migration lines, danger radii
    - `75%` (`4 km`) — directional drone icons, `RECONNAISSANCE` waypoint paths
    - `100%` (`2 000 m`) — tactical view: HP bars, threat/EVADE halos, firing lines
    - `125%` (`500 m`) — close combat: drone IDs, exact HP numbers, `Sensor Fusion` estimates
    - `150%` (`100 m`) — macro view: `30–40 m` `OBJECT_INSPECTION` approach & velocity vectors
- [ ] **Experimental Benchmark Scripts & Final Report Data**
  - Speedup curves (OpenMP 1, 2, 4, 8 threads).
  - Distribution overhead (Single-node vs 2-node).
  - Migration latency & recovery time graphs.
