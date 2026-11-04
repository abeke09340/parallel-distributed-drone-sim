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

## 🛠 Технологический стек проекта

```
+-----------------------------------------------------------------------------------+
| 1. C++20        | Чистая логика дронов, физика, сенсоры и миссии                  |
| 2. OpenMP       | #pragma omp parallel for — параллелим расчеты роя по ядрам ПК   |
| 3. OpenMPI      | Связь между нодами: передача дронов при миграции и синхронизация|
| 4. CMake        | Сборка всего проекта одной командой (make / ninja)              |
| 5. TCP/Sockets  | Легковесный стриминг кадров (JSON/координаты) из C++ наружу    |
| 6. Visualizer   | Экран: граница X=500, синие/зеленые дроны, живая миграция роя   |
+-----------------------------------------------------------------------------------+
```

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
├── CONTRIBUTING.md             # Git flow & team collaboration rules
├── ROADMAP.md                  # Milestone plan & progress tracker
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
│   ├── benchmark/
│   │   └── local_benchmark.cpp # Phase 1 standalone OpenMP benchmark runner
│   ├── comm/
│   ├── migration/
│   ├── fusion/
│   ├── task_allocation/
│   ├── fault_tolerance/
│   └── main.cpp                # Distributed MPI node entry point
└── visualization/              # Unity client or telemetry bridges
```

---

## 🚀 Quick Start Guide

### 1. Prerequisites

- **C++ Compiler:** MSVC 2022 (Windows), Clang 15+ (macOS), or GCC 11+ (Linux) with C++20 support
- **Build System:** CMake 3.20+
- **OpenMP Runtime:** Built-in MSVC/GCC OpenMP or `libomp` (macOS)
- **MPI Implementation (Optional for Phase 1 `local_benchmark`):** OpenMPI 4+ / MPICH / MS-MPI

#### macOS (Homebrew)
```bash
brew install cmake open-mpi libomp
```

#### Ubuntu / Debian
```bash
sudo apt update
sudo apt install build-essential cmake libopenmpi-dev openmpi-bin libomp-dev
```

### 2. Build & Run Phase 1 Local OpenMP Benchmark (Standalone, No MPI Required)

The `local_benchmark` target evaluates single-node parallel kinematics and noisy sensor scanning across multiple OpenMP thread counts and verifies thread-invariance via state checksums.

#### Windows (MSVC / Visual Studio 2022 Build Tools)
```powershell
cmake -B build
cmake --build build --config Release
.\build\Release\local_benchmark.exe --drones 100,500,1000 --threads 1,2,4,8 --steps 500
```

#### macOS / Linux
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
./build/local_benchmark --drones 100,500,1000 --threads 1,2,4,8 --steps 500
```

### 3. Run Distributed Mode Locally (2 MPI Simulation Nodes)

Requires MPI installed (`simulation_node` target is built automatically when CMake detects MPI):

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

- **Sync First:** Always run `git pull origin main` before starting work.
- **Verify Build:** Ensure CMake compiles cleanly before committing.
- **Task Tracking:** Check [ROADMAP.md](ROADMAP.md) for the active milestone progress.

---

## 📜 References

1. Paciorek and Turek, *Distribution-invariance in agent-based spatial simulations*, FGCS, 2026.
2. Mohan, Potturi, and Fukuda, *Automated Agent Migration over Distributed Data Structures*, ICAART, 2023.
3. Lajoie and Beltrame, *Swarm-SLAM: Sparse Decentralized Collaborative Simultaneous Localization and Mapping Framework for Multi-Robot Systems*, arXiv, 2024.
4. Wang, Wang, and Ren, *A Two-Level Clustered Consensus-Based Bundle Algorithm for Dynamic Heterogeneous Multi-UAV Multi-Task Allocation*, Sensors, 2025.
5. Zhou et al., *EGO-Swarm: A Fully Autonomous and Decentralized Quadrotor Swarm System in Cluttered Environments*, ICRA, 2021.
