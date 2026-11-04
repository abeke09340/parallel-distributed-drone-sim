#include "core/config.hpp"
#include "simulation/engine.hpp"

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <omp.h>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct BenchmarkOptions {
    std::vector<int> drone_counts{100, 500, 1000};
    std::vector<int> thread_counts{1, 2, 4, 8};
    int steps{500};
    std::string csv_path{};
};

std::vector<int> parse_int_list(const std::string& str) {
    std::vector<int> result;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (!item.empty()) {
            int val = std::stoi(item);
            if (val > 0) {
                result.push_back(val);
            }
        }
    }
    return result;
}

BenchmarkOptions parse_args(int argc, char** argv) {
    BenchmarkOptions opts;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--drones" && i + 1 < argc) {
            opts.drone_counts = parse_int_list(argv[++i]);
        } else if (arg == "--threads" && i + 1 < argc) {
            opts.thread_counts = parse_int_list(argv[++i]);
        } else if (arg == "--steps" && i + 1 < argc) {
            opts.steps = std::stoi(argv[++i]);
        } else if (arg == "--csv" && i + 1 < argc) {
            opts.csv_path = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: local_benchmark [options]\n"
                      << "  --drones <N1,N2,...>   Comma-separated drone counts (default: 100,500,1000)\n"
                      << "  --threads <T1,T2,...>  Comma-separated OpenMP thread counts (default: 1,2,4,8)\n"
                      << "  --steps <S>            Simulation steps per run (default: 500)\n"
                      << "  --csv <file.csv>       Optional path to export benchmark results as CSV\n";
            std::exit(0);
        }
    }
    return opts;
}

std::vector<sim::Vector3> generate_world_targets(int count = 25) {
    std::vector<sim::Vector3> targets;
    targets.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        float x = 100.0f + static_cast<float>((i * 173) % 800);
        float y = 100.0f + static_cast<float>((i * 257) % 800);
        float z = 15.0f + static_cast<float>((i * 43) % 120);
        targets.emplace_back(x, y, z);
    }
    return targets;
}

double compute_state_checksum(const std::vector<sim::Drone>& drones) {
    double sum = 0.0;
    for (const auto& d : drones) {
        const auto& s = d.get_state();
        sum += static_cast<double>(s.position.x) * 0.01
             + static_cast<double>(s.position.y) * 0.02
             + static_cast<double>(s.position.z) * 0.03
             + static_cast<double>(s.battery) * 0.1;
    }
    return sum;
}

}  // namespace

int main(int argc, char** argv) {
    BenchmarkOptions opts = parse_args(argc, argv);
    auto& config = sim::get_global_config();
    auto targets = generate_world_targets(25);

    std::cout << "====================================================================================\n";
    std::cout << " Phase 1: Local Parallel Simulation Engine & OpenMP Benchmark\n";
    std::cout << " Course: Parallel and Distributed Computing | Group 207\n";
    std::cout << " Hardware Max OpenMP Threads: " << omp_get_max_threads() << "\n";
    std::cout << " World Bounds: [" << config.world_width << " x "
              << config.world_length << " x " << config.world_height << "] m | "
              << "Targets: " << targets.size() << " | Steps/run: " << opts.steps << "\n";
    std::cout << "====================================================================================\n";

    std::ofstream csv_file;
    if (!opts.csv_path.empty()) {
        csv_file.open(opts.csv_path);
        if (csv_file.is_open()) {
            csv_file << "drones,threads,steps,total_ms,avg_step_ms,speedup,efficiency_pct,total_observations,checksum\n";
        }
    }

    bool all_checksums_valid = true;

    for (int num_drones : opts.drone_counts) {
        std::cout << "\n>>> Swarm Size: " << num_drones << " Autonomous Drones (" << opts.steps << " steps)\n";
        std::cout << "------------------------------------------------------------------------------------\n";
        std::cout << std::left
                  << std::setw(10) << "Threads"
                  << std::setw(15) << "Total (ms)"
                  << std::setw(15) << "Avg ms/step"
                  << std::setw(12) << "Speedup"
                  << std::setw(14) << "Efficiency"
                  << std::setw(14) << "Observations"
                  << "Checksum\n";
        std::cout << "------------------------------------------------------------------------------------\n";

        double baseline_ms = 0.0;
        double baseline_checksum = 0.0;

        for (size_t t_idx = 0; t_idx < opts.thread_counts.size(); ++t_idx) {
            int threads = opts.thread_counts[t_idx];
            config.openmp_threads = threads;
            omp_set_num_threads(threads);

            // Single-node mode (rank 0, 1 rank total covers full spatial domain)
            sim::SimulationEngine engine(0, 1);
            engine.initialize_agents(num_drones);
            engine.set_targets(targets);

            size_t total_observations = 0;

            auto t_start = std::chrono::high_resolution_clock::now();
            for (int step = 0; step < opts.steps; ++step) {
                engine.step(config.delta_time);
                total_observations += engine.collect_sensor_observations().size();
            }
            auto t_end = std::chrono::high_resolution_clock::now();

            double total_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            double avg_step_ms = total_ms / static_cast<double>(opts.steps);
            double checksum = compute_state_checksum(engine.get_drones());

            if (t_idx == 0) {
                baseline_ms = total_ms;
                baseline_checksum = checksum;
            } else if (std::abs(checksum - baseline_checksum) > 1e-4) {
                all_checksums_valid = false;
            }

            double speedup = (total_ms > 0.0) ? (baseline_ms / total_ms) : 1.0;
            double efficiency = (speedup / static_cast<double>(threads)) * 100.0;

            std::ostringstream speedup_ss, eff_ss;
            speedup_ss << std::fixed << std::setprecision(2) << speedup << "x";
            eff_ss << std::fixed << std::setprecision(1) << efficiency << "%";

            std::cout << std::left
                      << std::setw(10) << threads
                      << std::fixed << std::setprecision(2)
                      << std::setw(15) << total_ms
                      << std::setprecision(4)
                      << std::setw(15) << avg_step_ms
                      << std::setw(12) << speedup_ss.str()
                      << std::setw(14) << eff_ss.str()
                      << std::setw(14) << total_observations
                      << std::fixed << std::setprecision(4) << checksum << "\n";

            if (csv_file.is_open()) {
                csv_file << num_drones << ","
                         << threads << ","
                         << opts.steps << ","
                         << std::fixed << std::setprecision(3) << total_ms << ","
                         << std::setprecision(5) << avg_step_ms << ","
                         << std::setprecision(3) << speedup << ","
                         << std::setprecision(1) << efficiency << ","
                         << total_observations << ","
                         << std::setprecision(4) << checksum << "\n";
            }
        }
        std::cout << "------------------------------------------------------------------------------------\n";
    }

    std::cout << "\n[Verification] Thread-Invariance Checksum: "
              << (all_checksums_valid ? "PASSED (Identical state across all thread counts)"
                                      : "MISMATCH DETECTED")
              << "\n";
    std::cout << "====================================================================================\n";

    return all_checksums_valid ? 0 : 1;
}
