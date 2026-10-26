#pragma once

#include <vector>

#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"
#include "evrptw/Solver.hpp"

namespace evrptw {

// Granular first-improvement local search on the penalised cost. Routes are cached
// (simulation result and penalised cost per route, customer positions), so a move
// only re-simulates the one or two routes it modifies.
class LocalSearch {
public:
    static constexpr int kGranularity = 20;

    explicit LocalSearch(const Instance& inst);

    // Descends until no move improves the penalised cost. maxVehicles caps the number
    // of non-empty routes the descent may use (-1: no cap).
    void run(Solution& sol, const PenaltyWeights& w, Rng& rng, int maxVehicles = -1);

    // Same descent, but only the touched customers (and their neighbours) start active:
    // used after a local perturbation of an already optimised solution.
    void runFrom(Solution& sol, const PenaltyWeights& w, Rng& rng, int maxVehicles,
                 const std::vector<int>& touched);

    // Removes the shortest route, reinserts its customers by cheapest penalised
    // insertion, then descends with penalties escalated from `start` (x10 per attempt).
    // Updates sol and returns true only if a feasible solution with one vehicle fewer
    // is found; otherwise sol is left unchanged.
    bool tryEliminateRoute(Solution& sol, Rng& rng, const Timer& timer, double deadline,
                           const PenaltyWeights& start = PenaltyWeights{});

    // Nearest customers and stations of a node (granular neighbourhood).
    const std::vector<int>& neighbours(int node) const { return neighbours_[node]; }

private:
    void load(const Solution& sol);
    void descend(Solution& sol, const PenaltyWeights& w, Rng& rng, int maxVehicles,
                 const std::vector<int>* touched);
    void store(Solution& sol) const;
    void refreshRoute(int r);
    void setRoute(int r, Route& replacement);
    double costOf(const Route& route) const;
    int emptyRouteSlot();
    bool mayOpenRoute() const;

    bool improveCustomer(int u);
    bool relocate(int u);
    bool swapCustomers(int u);
    bool twoOptStar(int u);
    bool twoOpt(int u);
    bool stationRemoval();
    bool stationInsertion();

    const Instance& inst_;
    std::vector<std::vector<int>> neighbours_;         // per customer: nearest customers and stations
    std::vector<std::vector<int>> stationNeighbours_;  // per node: nearest stations
    std::vector<int> customers_;

    // Working state of the current run.
    PenaltyWeights w_;
    Rng* rng_ = nullptr;
    int maxVehicles_ = -1;
    std::vector<Route> routes_;
    std::vector<RouteEval> evals_;
    std::vector<double> penalty_;
    std::vector<int> routeOf_;  // per customer
    std::vector<int> posOf_;    // per customer
    std::vector<char> active_;  // per customer: "don't look" bit, cleared when no move was found
    int numNonEmpty_ = 0;
    Route bufA_, bufB_;         // candidate routes of the move being evaluated
};

}  // namespace evrptw
