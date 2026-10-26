#pragma once

#include "evrptw/Solver.hpp"

namespace evrptw {

// Exact MILP for the E-VRPTW following Schneider, Stenger & Goeke (2014), solved
// with Gurobi. Stations are duplicated `copiesPerStation` times (the station S0
// co-located with the depot included), so that a station can be visited more than
// once. Requires the build with WITH_GUROBI.
//
// Objective: if `hierarchical` (default), lexicographic (number of vehicles, then
// distance) via Gurobi multi-objective with priorities 2 and 1; otherwise distance
// only. SolveResult::lowerBound is the Gurobi ObjBound of the final objective
// solved, reported only when the solve ends OPTIMAL (in hierarchical mode this is
// a bound on the distance among fleet-minimal solutions, not on the distance alone)
// or, in non-hierarchical mode, at any termination (it is then a bound on distance).
class MilpSolver : public Solver {
public:
    explicit MilpSolver(int copiesPerStation = 2, bool hierarchical = true);

    std::string name() const override { return "milp"; }
    SolveResult solve(const Instance& inst, const SolverConfig& cfg) override;

private:
    int copiesPerStation_;
    bool hierarchical_;
};

}  // namespace evrptw
