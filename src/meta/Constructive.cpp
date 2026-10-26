#include "evrptw/meta/Constructive.hpp"

#include <algorithm>
#include <iostream>

namespace evrptw {

namespace {

constexpr double kEps = 1e-9;
constexpr int kMaxRepairSteps = 10;
constexpr int kMaxRepairTries = 25;

struct Slot {
    int route;
    int position;
    double distanceDelta;
};

// Inserts a node at slot `pos` of a route.
void insertAt(Route& route, int pos, int node) { route.insert(route.begin() + pos, node); }

}  // namespace

InsertionMove bestInsertion(const Instance& inst, const std::vector<Route>& routes, int customer,
                            const PenaltyWeights& w, bool requireFeasible, Route& scratch) {
    InsertionMove best;
    for (int r = 0; r < static_cast<int>(routes.size()); ++r) {
        const Route& route = routes[r];
        const double base = penalisedCost(evaluateRoute(inst, route), w);
        for (int p = 0; p <= static_cast<int>(route.size()); ++p) {
            scratch = route;
            insertAt(scratch, p, customer);
            const RouteEval e = evaluateRoute(inst, scratch);
            if (requireFeasible && !e.feasible()) continue;
            const double delta = penalisedCost(e, w) - base;
            if (delta < best.cost) best = {r, p, delta};
        }
    }
    return best;
}

bool repairWithStations(const Instance& inst, Route& route) {
    for (int step = 0; step < kMaxRepairSteps; ++step) {
        const RouteEval cur = evaluateRoute(inst, route);
        if (cur.feasible()) return true;
        const double curViolation = cur.violation();

        double bestViolation = std::numeric_limits<double>::infinity();
        double bestDistance = std::numeric_limits<double>::infinity();
        Route bestRoute;
        Route scratch;
        for (int s = inst.firstStation(); s < inst.firstCustomer(); ++s) {
            for (int p = 0; p <= static_cast<int>(route.size()); ++p) {
                scratch = route;
                insertAt(scratch, p, s);
                const RouteEval e = evaluateRoute(inst, scratch);
                const double v = e.violation();
                if (v < bestViolation - kEps ||
                    (v <= bestViolation + kEps && e.distance < bestDistance)) {
                    bestViolation = v;
                    bestDistance = e.distance;
                    bestRoute = scratch;
                }
            }
        }
        if (bestViolation >= curViolation - kEps) return false;
        route = std::move(bestRoute);
    }
    return evaluateRoute(inst, route).feasible();
}

int shortestRouteIndex(const Instance& inst, const Solution& sol) {
    int best = -1;
    double shortest = std::numeric_limits<double>::infinity();
    for (int r = 0; r < static_cast<int>(sol.routes.size()); ++r) {
        if (sol.routes[r].empty()) continue;
        const double d = evaluateRoute(inst, sol.routes[r]).distance;
        if (d < shortest) {
            shortest = d;
            best = r;
        }
    }
    return best;
}

bool dissolveRoute(const Instance& inst, Solution& sol, int index, const PenaltyWeights& w, Rng& rng) {
    if (index < 0 || sol.numVehicles() <= 1) return false;
    std::vector<int> orphans;
    for (int j : sol.routes[index])
        if (inst.isCustomer(j)) orphans.push_back(j);
    sol.routes.erase(sol.routes.begin() + index);
    std::shuffle(orphans.begin(), orphans.end(), rng);

    Route scratch;
    for (int c : orphans) {
        const InsertionMove m = bestInsertion(inst, sol.routes, c, w, false, scratch);
        if (m.route < 0) return false;
        insertAt(sol.routes[m.route], m.position, c);
    }
    return true;
}

Solution buildInitialSolution(const Instance& inst, Rng& rng) {
    std::vector<int> order;
    for (int c = inst.firstCustomer(); c < inst.numNodes(); ++c) order.push_back(c);
    std::shuffle(order.begin(), order.end(), rng);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return inst.node(a).due < inst.node(b).due;
    });

    Solution sol;
    Route scratch;
    for (int c : order) {
        const InsertionMove m = bestInsertion(inst, sol.routes, c, PenaltyWeights{}, true, scratch);
        if (m.route >= 0) {
            insertAt(sol.routes[m.route], m.position, c);
            continue;
        }

        // No feasible slot: try slots that only violate the battery and repair
        // them with charging stations, cheapest distance first.
        std::vector<Slot> batterySlots;
        for (int r = 0; r < static_cast<int>(sol.routes.size()); ++r) {
            const double base = evaluateRoute(inst, sol.routes[r]).distance;
            for (int p = 0; p <= static_cast<int>(sol.routes[r].size()); ++p) {
                scratch = sol.routes[r];
                insertAt(scratch, p, c);
                const RouteEval e = evaluateRoute(inst, scratch);
                if (e.capacityExcess <= 1e-6 && e.timeWarp <= 1e-6 && e.batteryDeficit > 1e-6)
                    batterySlots.push_back({r, p, e.distance - base});
            }
        }
        std::sort(batterySlots.begin(), batterySlots.end(), [](const Slot& a, const Slot& b) {
            return a.distanceDelta < b.distanceDelta;
        });
        bool placed = false;
        for (int t = 0; t < static_cast<int>(batterySlots.size()) && t < kMaxRepairTries && !placed; ++t) {
            Route candidate = sol.routes[batterySlots[t].route];
            insertAt(candidate, batterySlots[t].position, c);
            if (repairWithStations(inst, candidate)) {
                sol.routes[batterySlots[t].route] = std::move(candidate);
                placed = true;
            }
        }
        if (!placed) {
            // Open a new route, repaired with stations if it needs them.
            Route fresh{c};
            repairWithStations(inst, fresh);
            sol.routes.push_back(std::move(fresh));
        }
    }
    return sol;
}

SolveResult GreedySolver::solve(const Instance& inst, const SolverConfig& cfg) {
    Timer timer;
    Rng rng(cfg.seed);
    SolveResult result;
    result.best = buildInitialSolution(inst, rng);
    result.feasible = result.best.feasible(inst);
    result.runtime = timer.elapsed();
    result.trace.push_back({result.runtime, result.best.numVehicles(), result.best.distance(inst)});
    if (cfg.verbose)
        std::cerr << "[greedy] vehicles=" << result.best.numVehicles()
                  << " distance=" << result.best.distance(inst) << " feasible=" << result.feasible << '\n';
    return result;
}

}  // namespace evrptw
