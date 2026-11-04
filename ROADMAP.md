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
**Objective:** Two nodes exchanging drone state dynamically when crossing the boundary.

- [ ] **MPI Lifecycle Manager** (`mpi_manager.cpp`)
  - Initialize MPI, rank identification (Rank 0 = Region A, Rank 1 = Region B).
- [ ] **Message Protocol & Binary Serialization** (`protocol.hpp`)
  - Packed structs or serialization buffers for MPI send/recv.
- [ ] **2-Phase Migration Protocol** (`migration_manager.cpp`)
  - Detect $X \ge 500.0$, serialize drone, send `MSG_MIGRATION_REQ`, wait for `MSG_MIGRATION_ACK`.
- [ ] **Target Node Ingestion & De-duplication**
  - Receive drone payload on Rank 1, instantiate local drone, send ACK.
- [ ] **Migration Latency & Zero-Duplicate Verification Tests**

---

## Phase 3: Sensor Fusion & Distributed Task Allocation
**Objective:** Multi-node perception and decentralized mission planning.

- [ ] **Decentralized Track Manager** (`sensor_fusion.cpp`)
  - Multi-observation clustering for simulated target objects.
- [ ] **Time- & Confidence-Weighted Fusion**
  - Equation: $\hat{P} = \sum w_i P_i / \sum w_i$ with exponential time decay.
- [ ] **Distributed Task Auction & Coordination** (`task_coordinator.cpp`)
  - Dynamic missions (Search, Track, Inspect).
  - Bid evaluation: $U = f(\text{dist}, \text{battery}, \text{workload})$.
- [ ] **Consensus Resolution over MPI** (`MSG_TASK_ANNOUNCE`, `MSG_TASK_CLAIM`)

---

## Phase 4: Fault Tolerance, Evaluation & Visualization
**Objective:** Node failure recovery, network partition tolerance, and final evaluation.

- [ ] **Heartbeat Monitor & Timeout Handler** (`fault_manager.cpp`)
  - Periodic ping/pong, detect missing rank after threshold.
- [ ] **State Checkpointing & Asynchronous Replication**
  - Periodic snapshot of active mission tracks and border states.
- [ ] **Split-Brain & Partition Reconciliation**
  - Simulate network cut (`mpirun` partition injection), resume and reconcile sequence logs.
- [ ] **Telemetry Bridge for Visualization** (`bridge.cpp`)
  - Lightweight TCP/WebSocket server broadcasting JSON frames to Unity / Web Viewer.
- [ ] **Experimental Benchmark Scripts & Final Report Data**
  - Speedup curves (OpenMP 1, 2, 4, 8 threads).
  - Distribution overhead (Single-node vs 2-node).
  - Migration latency & recovery time graphs.
