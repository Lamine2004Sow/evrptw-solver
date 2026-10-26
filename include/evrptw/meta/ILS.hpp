#pragma once

#include "evrptw/Solver.hpp"

namespace evrptw {

// Two-phase Iterated Local Search. Phase A minimises the fleet by route elimination;
// phase B minimises distance with ruin & recreate, granular local search and
// adaptive penalties. Deterministic for a given seed, up to the time limit.
class IteratedLocalSearch : public Solver {
public:
    std::string name() const override { return "ils"; }
    SolveResult solve(const Instance& inst, const SolverConfig& cfg) override;
};

}  // namespace evrptw
