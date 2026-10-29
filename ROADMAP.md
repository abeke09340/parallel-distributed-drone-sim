# Project Roadmap & Task Distribution

This roadmap breaks down the development of the **Distributed Multi-Agent Simulation** into 4 structured milestones. Tasks are assigned between **Abdirakhman** and **Meiirlan** according to their focus areas.

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

- [ ] **Project Setup & CMake scaffolding** (`CMakeLists.txt`, headers, git setup) — *Joint*
- [ ] **Drone agent data structures & kinematics** (`types.hpp`, `drone.cpp`) — *Abdirakhman*
  - Position, velocity, battery drain, waypoint steering.
- [ ] **Spatial Region & OpenMP Engine** (`engine.cpp`) — *Abdirakhman*
  - Parallel loop (`#pragma omp parallel for`) for 500+ drones at 60 Hz.
- [ ] **Sensory Field-of-View & Target generation** (`drone.cpp`, `sensor_fusion.hpp`) — *Meiirlan*
  - Distance checks, noise simulation, observation records.
- [ ] **Basic CLI telemetry & benchmarks** (measuring tick execution time vs thread count) — *Meiirlan*

---

## Phase 2: Distributed MPI & Agent Migration
**Objective:** Two nodes exchanging drone state dynamically when crossing the boundary.

- [ ] **MPI Lifecycle Manager** (`mpi_manager.cpp`) — *Abdirakhman*
  - Initialize MPI, rank identification (Rank 0 = Region A, Rank 1 = Region B).
- [ ] **Message Protocol & Binary Serialization** (`protocol.hpp`) — *Joint*
  - Packed structs or serialization buffers for MPI send/recv.
- [ ] **2-Phase Migration Protocol** (`migration_manager.cpp`) — *Abdirakhman*
  - Detect $X \ge 500.0$, serialize drone, send `MSG_MIGRATION_REQ`, wait for `MSG_MIGRATION_ACK`.
- [ ] **Target Node Ingestion & De-duplication** — *Meiirlan*
  - Receive drone payload on Rank 1, instantiate local drone, send ACK.
- [ ] **Migration Latency & Zero-Duplicate Verification Tests** — *Joint*

---

## Phase 3: Sensor Fusion & Distributed Task Allocation
**Objective:** Multi-node perception and decentralized mission planning.

- [ ] **Decentralized Track Manager** (`sensor_fusion.cpp`) — *Meiirlan*
  - Multi-observation clustering for simulated target objects.
- [ ] **Time- & Confidence-Weighted Fusion** — *Meiirlan*
  - Equation: $\hat{P} = \sum w_i P_i / \sum w_i$ with exponential time decay.
- [ ] **Distributed Task Auction & Coordination** (`task_coordinator.cpp`) — *Meiirlan*
  - Dynamic missions (Search, Track, Inspect).
  - Bid evaluation: $U = f(\text{dist}, \text{battery}, \text{workload})$.
- [ ] **Consensus Resolution over MPI** (`MSG_TASK_ANNOUNCE`, `MSG_TASK_CLAIM`) — *Abdirakhman*

---

## Phase 4: Fault Tolerance, Evaluation & Visualization
**Objective:** Node failure recovery, network partition tolerance, and final evaluation.

- [ ] **Heartbeat Monitor & Timeout Handler** (`fault_manager.cpp`) — *Abdirakhman*
  - Periodic ping/pong, detect missing rank after threshold.
- [ ] **State Checkpointing & Asynchronous Replication** — *Abdirakhman*
  - Periodic snapshot of active mission tracks and border states.
- [ ] **Split-Brain & Partition Reconciliation** — *Joint*
  - Simulate network cut (`mpirun` partition injection), resume and reconcile sequence logs.
- [ ] **Telemetry Bridge for Visualization** (`bridge.cpp`) — *Meiirlan*
  - Lightweight TCP/WebSocket server broadcasting JSON frames to Unity / Web Viewer.
- [ ] **Experimental Benchmark Scripts & Final Report Data** — *Joint*
  - Speedup curves (OpenMP 1, 2, 4, 8 threads).
  - Distribution overhead (Single-node vs 2-node).
  - Migration latency & recovery time graphs.
