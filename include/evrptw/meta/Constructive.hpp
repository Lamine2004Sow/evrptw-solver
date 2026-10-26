#pragma once

#include <limits>
#include <vector>

#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"
#include "evrptw/Solver.hpp"

namespace evrptw {

// Weighted violations added to the distance: the penalised cost used by the
// local search and the ILS. Equal to the distance on a feasible route.
inline double penalisedCost(const RouteEval& e, const PenaltyWeights& w) {
    return e.distance + w.capacity * e.capacityExcess + w.time * e.timeWarp +
           w.battery * e.batteryDeficit;
}

// Best position found for one customer: route index, insertion slot and the
// increase of penalised cost it causes. route == -1 means no candidate.
struct InsertionMove {
    int route = -1;
    int position = -1;
    double cost = std::numeric_limits<double>::infinity();
};

// Scans every slot of every route. With requireFeasible, only slots that keep the
// route feasible are considered. `scratch` is a reusable buffer (no allocation in the loop).
InsertionMove bestInsertion(const Instance& inst, const std::vector<Route>& routes, int customer,
                            const PenaltyWeights& w, bool requireFeasible, Route& scratch);

// Greedy repair: inserts charging stations, one at a time, while the route
// violates battery constraints and each insertion lowers the violation.
// Returns true if the route ends up feasible.
bool repairWithStations(const Instance& inst, Route& route);

// Index of the non-empty route with the smallest distance, -1 if there is none.
int shortestRouteIndex(const Instance& inst, const Solution& sol);

// Removes route `index` and reinserts its customers, in random order, by cheapest
// penalised insertion into the remaining routes (no new route is opened).
// Returns false if there is no other route to receive them.
bool dissolveRoute(const Instance& inst, Solution& sol, int index, const PenaltyWeights& w, Rng& rng);

// Sequential cheapest feasible insertion, customers by increasing due date
// (random tie-breaking). Always returns a feasible solution on the benchmark.
Solution buildInitialSolution(const Instance& inst, Rng& rng);

class GreedySolver : public Solver {
public:
    std::string name() const override { return "greedy"; }
    SolveResult solve(const Instance& inst, const SolverConfig& cfg) override;
};

}  // namespace evrptw
