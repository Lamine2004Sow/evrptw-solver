#ifdef EVRPTW_WITH_GUROBI

#include <gtest/gtest.h>

#include <string>

#include "evrptw/Checker.hpp"
#include "evrptw/Instance.hpp"
#include "evrptw/milp/MilpSolver.hpp"

using namespace evrptw;

TEST(MilpSolver, SolvesC101C5ToOptimality) {
    Instance inst = Instance::load(std::string(EVRPTW_DATA_DIR) + "/c101C5.txt");
    SolverConfig cfg;
    cfg.timeLimit = 60;
    cfg.seed = 1;

    MilpSolver solver;
    SolveResult res = solver.solve(inst, cfg);

    EXPECT_TRUE(res.feasible);
    EXPECT_TRUE(res.optimal);
    CheckReport rep = checkSolution(inst, res.best);
    EXPECT_TRUE(rep.ok) << (rep.errors.empty() ? std::string("no details") : rep.errors.front());
}

#endif  // EVRPTW_WITH_GUROBI
