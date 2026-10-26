#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"

namespace evrptw {

using Rng = std::mt19937_64;

class Timer {
public:
    Timer() : start_(std::chrono::steady_clock::now()) {}
    double elapsed() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_).count();
    }

private:
    std::chrono::steady_clock::time_point start_;
};

struct SolverConfig {
    std::uint64_t seed = 1;
    double timeLimit = 10.0;  // seconds, wall clock
    int maxVehicles = -1;     // optional fleet-size cap (epsilon-constraint); -1 = none
    bool verbose = false;
};

// One point of the convergence trace: best feasible solution found so far.
struct TracePoint {
    double time;
    int vehicles;
    double distance;
};

struct SolveResult {
    Solution best;
    bool feasible = false;
    double runtime = 0;
    double lowerBound = -1;  // exact methods only
    bool optimal = false;    // exact methods only
    std::vector<TracePoint> trace;
};

// Common interface of every method (exact or heuristic), so the experiment
// runner and the CLI can treat them uniformly.
class Solver {
public:
    virtual ~Solver() = default;
    virtual std::string name() const = 0;
    virtual SolveResult solve(const Instance& inst, const SolverConfig& cfg) = 0;
};

// Factory: "greedy", "ils", "ga", "milp" (if built WITH_GUROBI).
std::unique_ptr<Solver> makeSolver(const std::string& name);
std::vector<std::string> availableSolvers();

}  // namespace evrptw
