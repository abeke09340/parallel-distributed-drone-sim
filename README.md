# Distributed Multi-Agent Simulation with Sensor Fusion, Agent Migration, and Fault Tolerance

[![C++20](https://img.shields.io/badge/C%2B%2B-20-blue.svg)](https://en.cppreference.com/w/cpp/20)
[![MPI](https://img.shields.io/badge/MPI-OpenMPI%20%2F%20MPICH-orange.svg)](https://www.open-mpi.org/)
[![OpenMP](https://img.shields.io/badge/OpenMP-Multi--Threading-green.svg)](https://www.openmp.org/)
[![CMake](https://img.shields.io/badge/CMake-3.20%2B-brightgreen.svg)](https://cmake.org/)

**Course:** Parallel and Distributed Computing  
**Group:** 207  
**Authors:**
- **Abdirakhman Tangirbergen** ([@AbdurakhmanT](https://github.com/))
- **Meiirlan Meirambek**

---

## 📌 Project Overview

This project implements a high-performance, fault-tolerant distributed multi-agent simulation where a shared virtual environment is partitioned across multiple physical computing nodes (or MPI processes). Each node owns a designated spatial region and simulates hundreds of autonomous drones and virtual sensors concurrently.

### Key Capabilities
- **Spatial Partitioning & Invariance:** Virtual 2D/3D environment split between Node A (Region A) and Node B (Region B).
- **Dynamic Agent Migration:** Autonomous drones crossing region boundaries seamlessly migrate their complete state (coordinates, velocity, mission, battery, sensor log) across physical machines via MPI serialization without duplication or state loss.
- **Hybrid Parallel & Distributed Model:**
  - **OpenMP** inside each machine for shared-memory parallel processing of hundreds of drones and sensor calculations per step.
  - **MPI** across machines for distributed state coordination, migration protocols, and fault heartbeats.
- **Decentralized Sensor Fusion:** Multi-drone and multi-node sensor observations of simulated targets combined using confidence- and time-weighted fusion without a single centralized bottleneck.
- **Distributed Task Allocation:** Decentralized mission auction / consensus coordination rather than a single scheduler.
- **Fault Tolerance & Recovery:** Active heartbeat monitoring, checkpointing/replication of critical state, split-brain tolerance during partitions, and automatic reconciliation upon node reconnection.
- **Visualization Bridge:** Decoupled TCP/WebSocket telemetry interface transmitting real-time state snapshots to Unity or a dashboard without slowing down the simulation loop.

---

## 🏗 System Architecture

```
                    +------------------------------------+
                    |       Unity / Web Visualizer       |
                    +-----------------+------------------+
                                      ^ (TCP / WebSockets)
                                      |
         +----------------------------+----------------------------+
         |                                                         |
+--------v-------------------------+     MPI Communication     +----v-------------------------+
|     Node A (Region A)            |<=========================>|     Node B (Region B)        |
| - Simulation Engine (OpenMP)     |   - Agent Migration       | - Simulation Engine (OpenMP) |
| - Sensor Layer & Fusion Manager  |   - Sensor Observations   | - Sensor Layer & Fusion      |
| - Task Allocation (Auction logic)|   - Task Consensus        | - Task Allocation            |
| - Fault Manager (Heartbeats/Sync)|   - Heartbeats / Replicas | - Fault Manager              |
+----------------------------------+                           +------------------------------+
```

For in-depth diagrams and message formats, see [ARCHITECTURE.md](ARCHITECTURE.md).

---

## 📂 Repository Structure

```
.
├── CMakeLists.txt              # Top-level CMake build definition
├── README.md                   # Project description and quickstart
├── ARCHITECTURE.md             # In-depth architectural specification
├── CONTRIBUTING.md             # Git flow, PR workflow & team conventions
├── ROADMAP.md                  # Milestone plan & team task breakdown
├── .gitignore                  # Git exclusions for C++, CMake, Unity
├── .clang-format               # Code formatting rules (Google style C++20)
├── scripts/
│   ├── run_local.sh            # Run 2 simulation nodes locally via MPI
│   └── setup_env.sh            # Setup dependencies on macOS / Linux
├── include/
│   ├── core/
│   │   ├── config.hpp          # Global simulation parameters
│   │   └── types.hpp           # Data structures (Drone, Sensor, Task, Event)
│   ├── simulation/
│   │   ├── drone.hpp           # Autonomous drone agent model
│   │   └── engine.hpp          # Simulation engine & OpenMP updates
│   ├── comm/
│   │   ├── mpi_manager.hpp     # MPI lifecycle & non-blocking communications
│   │   └── protocol.hpp        # Message types, serialization tags & structs
│   ├── migration/
│   │   └── migration_manager.hpp # 2-phase migration protocol
│   ├── fusion/
│   │   └── sensor_fusion.hpp   # Decentralized track fusion
│   ├── task_allocation/
│   │   └── task_coordinator.hpp# Distributed task auction & consensus
│   └── fault_tolerance/
│       └── fault_manager.hpp   # Heartbeats, checkpoints & recovery
├── src/
│   ├── core/
│   ├── simulation/
│   ├── comm/
│   ├── migration/
│   ├── fusion/
│   ├── task_allocation/
│   ├── fault_tolerance/
│   └── main.cpp                # Node entry point
└── visualization/              # Unity client or telemetry bridges
```

---

## 🚀 Quick Start Guide

### 1. Prerequisites

- **C++ Compiler:** Clang 15+ or GCC 11+ with C++20 support
- **Build System:** CMake 3.20+
- **MPI Implementation:** OpenMPI 4+ or MPICH
- **OpenMP Runtime:** `libomp` (macOS) or built-in GCC OpenMP (Linux)

#### macOS (Homebrew)
```bash
brew install cmake open-mpi libomp
```

#### Ubuntu / Debian
```bash
sudo apt update
sudo apt install build-essential cmake libopenmpi-dev openmpi-bin libomp-dev
```

### 2. Build the Project

```bash
# Clone the repository
git clone https://github.com/<your-username>/parallel-distributed-drone-sim.git
cd parallel-distributed-drone-sim

# Generate build files and compile
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

### 3. Run Locally (2 Simulation Nodes)

```bash
# Using helper script
./scripts/run_local.sh

# Or directly with mpirun
mpirun -np 2 ./build/simulation_node
```

### 4. Run Across Two Physical Computers

1. Connect both computers to the same local network (LAN / Wi-Fi).
2. Configure SSH keys between both machines for passwordless login.
3. Create a `hostfile`:
   ```
   192.168.1.10 slots=1
   192.168.1.11 slots=1
   ```
4. Run:
   ```bash
   mpirun --hostfile hostfile -np 2 ./build/simulation_node
   ```

---

## 👥 Team Collaboration & Workflow

Please read [CONTRIBUTING.md](CONTRIBUTING.md) before pushing code!

- **Branching:** Work on feature branches (`feature/migration`, `feature/fusion`), never directly push untested code to `main`.
- **Pull Requests:** Open a PR, review each other's changes, verify clean build, and squash/merge.
- **Task Tracking:** Check [ROADMAP.md](ROADMAP.md) for the active milestone assignments.

---

## 📜 References

1. Paciorek and Turek, *Distribution-invariance in agent-based spatial simulations*, FGCS, 2026.
2. Mohan, Potturi, and Fukuda, *Automated Agent Migration over Distributed Data Structures*, ICAART, 2023.
3. Lajoie and Beltrame, *Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems*, arXiv, 2024.
4. Wang, Wang, and Ren, *A Two-Level Clustered Consensus-Based Bundle Algorithm for Dynamic Heterogeneous Multi-UAV Multi-Task Allocation*, Sensors, 2025.
5. Zhou et al., *EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments*, ICRA, 2021.
