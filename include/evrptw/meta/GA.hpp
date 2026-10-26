#pragma once

#include <string>
#include <vector>

#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"
#include "evrptw/Solver.hpp"

namespace evrptw {

// Prins-style split of a giant tour (a permutation of the customers) into routes.
// A shortest path over the sequence: each segment is one route, costed by its
// penalised cost, so every tour decodes to a Solution, feasible or not. Segments are
// capped at kMaxRouteCustomers customers, which keeps the decoder O(n * L). Stations
// are left out here; the local search inserts them.
class Split {
public:
    static constexpr int kMaxRouteCustomers = 25;

    explicit Split(const Instance& inst) : inst_(inst) {}

    Solution decode(const std::vector<int>& tour, const PenaltyWeights& w) const;

    // Giant tour of a solution: its customers in route order, stations dropped.
    static std::vector<int> encode(const Solution& sol, const Instance& inst);

private:
    const Instance& inst_;
};

// Memetic algorithm in the spirit of HGS (Vidal 2012): giant-tour chromosomes,
// split decoding, local search education, OX crossover, biased-fitness survivor
// selection, adaptive penalties and periodic fleet minimisation.
class MemeticGA : public Solver {
public:
    std::string name() const override { return "ga"; }
    SolveResult solve(const Instance& inst, const SolverConfig& cfg) override;
};

}  // namespace evrptw
