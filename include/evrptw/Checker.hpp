#pragma once

#include <string>
#include <vector>

#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"

namespace evrptw {

struct CheckReport {
    bool ok = true;
    int vehicles = 0;
    double distance = 0;
    std::vector<std::string> errors;
};

// Independent feasibility checker. It deliberately does NOT reuse evaluateRoute():
// it re-simulates every route strictly (no time warp, no clamping) so that a bug
// in the search-side evaluation cannot hide itself. Every result written by the
// experiment pipeline must pass this check.
CheckReport checkSolution(const Instance& inst, const Solution& sol, double eps = 1e-6);

}  // namespace evrptw
