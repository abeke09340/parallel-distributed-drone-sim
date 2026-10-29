#!/usr/bin/env bash
set -e

# Distributed Multi-Agent Simulation: Local 2-Node Runner
# Runs 2 MPI ranks on the local machine (Rank 0 = Region A, Rank 1 = Region B)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(dirname "$SCRIPT_DIR")"

echo "=========================================================="
echo " Building Distributed Simulation Engine (CMake)..."
echo "=========================================================="
cd "$ROOT_DIR"

cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j

echo ""
echo "=========================================================="
echo " Launching 2 Simulation Nodes with MPI..."
echo " Node 0 -> Region A [0.0, 500.0)"
echo " Node 1 -> Region B [500.0, 1000.0]"
echo "=========================================================="

# Run 2 processes locally
mpirun -np 2 ./build/simulation_node "$@"
