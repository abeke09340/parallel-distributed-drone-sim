cmake_minimum_required(VERSION 3.20)
project(DistributedDroneSim
    VERSION 1.0.0
    DESCRIPTION "Distributed Multi-Agent Simulation with Sensor Fusion, Agent Migration, and Fault Tolerance"
    LANGUAGES C CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# -----------------------------------------------------------------------------
# Compiler Flags & Warnings
# -----------------------------------------------------------------------------
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang|GNU")
    add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# -----------------------------------------------------------------------------
# OpenMP Support (Handles macOS Homebrew libomp automatically)
# -----------------------------------------------------------------------------
if(APPLE)
    # Check Homebrew libomp location on Apple Silicon / Intel
    if(EXISTS "/opt/homebrew/opt/libomp")
        set(OpenMP_ROOT "/opt/homebrew/opt/libomp")
    elseif(EXISTS "/usr/local/opt/libomp")
        set(OpenMP_ROOT "/usr/local/opt/libomp")
    endif()
endif()

find_package(OpenMP REQUIRED)

# -----------------------------------------------------------------------------
# MPI Support
# -----------------------------------------------------------------------------
find_package(MPI REQUIRED)

# -----------------------------------------------------------------------------
# Threads Support
# -----------------------------------------------------------------------------
find_package(Threads REQUIRED)

# -----------------------------------------------------------------------------
# Include Directories & Sources
# -----------------------------------------------------------------------------
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MPI_CXX_INCLUDE_PATH}
)

set(SIMULATION_SOURCES
    src/core/config.cpp
    src/simulation/drone.cpp
    src/simulation/engine.cpp
    src/comm/mpi_manager.cpp
    src/migration/migration_manager.cpp
    src/fusion/sensor_fusion.cpp
    src/task_allocation/task_coordinator.cpp
    src/fault_tolerance/fault_manager.cpp
    src/main.cpp
)

# -----------------------------------------------------------------------------
# Executable Targets
# -----------------------------------------------------------------------------
add_executable(simulation_node ${SIMULATION_SOURCES})

target_link_libraries(simulation_node PRIVATE
    MPI::MPI_CXX
    OpenMP::OpenMP_CXX
    Threads::Threads
)

target_include_directories(simulation_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Optional: Add testing if enabled
enable_testing()
message(STATUS "Configured Distributed Drone Simulation successfully!")
message(STATUS "Compiler: ${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "MPI CXX Compiler: ${MPI_CXX_COMPILER}")
message(STATUS "OpenMP CXX flags: ${OpenMP_CXX_FLAGS}")
