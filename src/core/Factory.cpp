#include <stdexcept>

#include "evrptw/Solver.hpp"

// Each method lives in its own module; the factory only registers the ones that
// are present in the build (the MILP is compiled only WITH_GUROBI).
#if __has_include("evrptw/meta/Constructive.hpp")
#include "evrptw/meta/Constructive.hpp"
#define EVRPTW_HAS_GREEDY
#endif
#if __has_include("evrptw/meta/ILS.hpp")
#include "evrptw/meta/ILS.hpp"
#define EVRPTW_HAS_ILS
#endif
#if __has_include("evrptw/meta/GA.hpp")
#include "evrptw/meta/GA.hpp"
#define EVRPTW_HAS_GA
#endif
#if defined(EVRPTW_WITH_GUROBI) && __has_include("evrptw/milp/MilpSolver.hpp")
#include "evrptw/milp/MilpSolver.hpp"
#define EVRPTW_HAS_MILP
#endif

namespace evrptw {

std::unique_ptr<Solver> makeSolver(const std::string& name) {
#ifdef EVRPTW_HAS_GREEDY
    if (name == "greedy") return std::make_unique<GreedySolver>();
#endif
#ifdef EVRPTW_HAS_ILS
    if (name == "ils") return std::make_unique<IteratedLocalSearch>();
#endif
#ifdef EVRPTW_HAS_GA
    if (name == "ga") return std::make_unique<MemeticGA>();
#endif
#ifdef EVRPTW_HAS_MILP
    if (name == "milp") return std::make_unique<MilpSolver>();
#endif
    throw std::invalid_argument("unknown or unavailable solver: " + name);
}

std::vector<std::string> availableSolvers() {
    std::vector<std::string> v;
#ifdef EVRPTW_HAS_GREEDY
    v.push_back("greedy");
#endif
#ifdef EVRPTW_HAS_ILS
    v.push_back("ils");
#endif
#ifdef EVRPTW_HAS_GA
    v.push_back("ga");
#endif
#ifdef EVRPTW_HAS_MILP
    v.push_back("milp");
#endif
    return v;
}

}  // namespace evrptw
