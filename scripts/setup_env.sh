#!/usr/bin/env bash
set -e

# Setup script for dependencies (macOS & Linux)

echo "Detecting operating system..."
OS="$(uname -s)"

if [ "$OS" = "Darwin" ]; then
    echo "macOS detected. Checking Homebrew..."
    if ! command -v brew &> /dev/null; then
        echo "Error: Homebrew is not installed. Please install Homebrew first: https://brew.sh/"
        exit 1
    fi
    echo "Installing CMake, Open-MPI, libomp, and GitHub CLI via brew..."
    brew install cmake open-mpi libomp gh
    echo "Setup complete for macOS!"
elif [ "$OS" = "Linux" ]; then
    echo "Linux detected. Checking apt..."
    if command -v apt-get &> /dev/null; then
        echo "Installing build-essential, CMake, OpenMPI, and libomp via apt..."
        sudo apt-get update
        sudo apt-get install -y build-essential cmake libopenmpi-dev openmpi-bin libomp-dev gh
        echo "Setup complete for Linux (Debian/Ubuntu)!"
    else
        echo "Please install CMake, OpenMPI (or MPICH), OpenMP development libraries with your distro package manager."
    fi
else
    echo "Unsupported OS: $OS"
fi
