#pragma once

#include <string>
#include <vector>

#include "evrptw/Instance.hpp"

namespace evrptw {

// A route is the sequence of visited nodes (customers and stations), without the
// depot at either end: the vehicle implicitly leaves from and returns to node 0.
using Route = std::vector<int>;

// Result of simulating one route. Violations are measured, not rejected, so the
// same evaluation drives both penalised local search and strict checking.
struct RouteEval {
    double distance = 0;
    double load = 0;
    double capacityExcess = 0;   // load - C, if positive
    double timeWarp = 0;         // total lateness, time-warp style (Vidal et al. 2013)
    double batteryDeficit = 0;   // total missing energy over the route
    double duration = 0;         // return time at the depot

    double violation() const { return capacityExcess + timeWarp + batteryDeficit; }
    bool feasible(double eps = 1e-6) const {
        return capacityExcess <= eps && timeWarp <= eps && batteryDeficit <= eps;
    }
};

// Simulates a route under the Schneider model: linear consumption, full linear
// recharge at stations, waiting allowed, late arrival absorbed as time warp.
RouteEval evaluateRoute(const Instance& inst, const Route& route);

struct PenaltyWeights {
    double capacity = 10.0;
    double time = 10.0;
    double battery = 10.0;
};

class Solution {
public:
    std::vector<Route> routes;

    // Number of non-empty routes.
    int numVehicles() const;
    double distance(const Instance& inst) const;
    // distance + weighted violations; equal to distance() when feasible.
    double penalizedCost(const Instance& inst, const PenaltyWeights& w) const;
    bool feasible(const Instance& inst) const;

    void removeEmptyRoutes();

    // Lexicographic comparison (vehicles, then distance), the objective of Schneider et al.
    static bool lexBetter(int vehA, double distA, int vehB, double distB, double eps = 1e-7);

    // Text format: one route per line, node ids separated by spaces, depot included,
    // e.g. "D0 C12 S3 C30 D0". Lines starting with '#' are comments.
    void write(const Instance& inst, const std::string& path) const;
    std::string toString(const Instance& inst) const;
    static Solution read(const Instance& inst, const std::string& path);
};

}  // namespace evrptw
