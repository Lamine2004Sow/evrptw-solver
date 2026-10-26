#pragma once

#include "evrptw/Solver.hpp"

namespace evrptw {

// Exact MILP for the E-VRPTW following Schneider, Stenger & Goeke (2014), solved
// with Gurobi. Stations are duplicated `copiesPerStation` times (the station S0
// co-located with the depot included), so that a station can be visited more than
// once. Requires the build with WITH_GUROBI.
//
// Caveat: "optimal" means optimal for the given number of copies. With 2 copies,
// rc201C10 is "solved" at 414.81 while the ILS finds 412.86 using S0 three times;
// hence the default of 3. Options (SolverConfig::options): copies=N overrides the
// number of copies; warmstart=T runs the ILS for T seconds and passes its solution
// to Gurobi as a MIP start.
//
// Objective: if `hierarchical` (default), lexicographic (number of vehicles, then
// distance) via Gurobi multi-objective with priorities 2 and 1; otherwise distance
// only. SolveResult::lowerBound is the Gurobi ObjBound of the final objective
// solved, reported only when the solve ends OPTIMAL (in hierarchical mode this is
// a bound on the distance among fleet-minimal solutions, not on the distance alone)
// or, in non-hierarchical mode, at any termination (it is then a bound on distance).
class MilpSolver : public Solver {
public:
    explicit MilpSolver(int copiesPerStation = 3, bool hierarchical = true);

    std::string name() const override { return "milp"; }
    SolveResult solve(const Instance& inst, const SolverConfig& cfg) override;

private:
    int copiesPerStation_;
    bool hierarchical_;
};

}  // namespace evrptw
