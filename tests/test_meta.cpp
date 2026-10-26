#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "evrptw/Checker.hpp"
#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"
#include "evrptw/Solver.hpp"
#include "evrptw/meta/Constructive.hpp"
#include "evrptw/meta/ILS.hpp"

using namespace evrptw;

namespace {

std::string instancePath(const std::string& name) {
    return std::string(EVRPTW_DATA_DIR) + "/" + name + ".txt";
}

// Every customer must be served exactly once.
void expectEveryCustomerOnce(const Instance& inst, const Solution& sol) {
    std::vector<int> visits(inst.numNodes(), 0);
    for (const Route& r : sol.routes)
        for (int j : r)
            if (inst.isCustomer(j)) ++visits[j];
    for (int c = inst.firstCustomer(); c < inst.numNodes(); ++c) EXPECT_EQ(visits[c], 1) << "customer " << c;
}

void expectCheckerOk(const Instance& inst, const Solution& sol) {
    const CheckReport rep = checkSolution(inst, sol);
    for (const auto& e : rep.errors) ADD_FAILURE() << e;
    EXPECT_TRUE(rep.ok);
}

}  // namespace

TEST(Greedy, FeasibleOnLargeInstance) {
    Instance inst = Instance::load(instancePath("c101C10"));
    Rng rng(1);
    Solution sol = buildInitialSolution(inst, rng);
    expectCheckerOk(inst, sol);
    expectEveryCustomerOnce(inst, sol);
}

TEST(Greedy, FeasibleOnSmallInstance) {
    Instance inst = Instance::load(instancePath("r101_21"));
    Rng rng(7);
    Solution sol = buildInitialSolution(inst, rng);
    expectCheckerOk(inst, sol);
    expectEveryCustomerOnce(inst, sol);
}

TEST(Greedy, SolverReportsFeasible) {
    Instance inst = Instance::load(instancePath("rc101_21"));
    SolverConfig cfg;
    cfg.seed = 3;
    GreedySolver solver;
    SolveResult res = solver.solve(inst, cfg);
    EXPECT_TRUE(res.feasible);
    expectCheckerOk(inst, res.best);
}

class IlsTest : public ::testing::TestWithParam<std::string> {};

TEST_P(IlsTest, NoWorseThanGreedyAndFeasible) {
    Instance inst = Instance::load(instancePath(GetParam()));
    Rng rng(1);
    Solution greedy = buildInitialSolution(inst, rng);

    SolverConfig cfg;
    cfg.seed = 1;
    cfg.timeLimit = 3.0;
    IteratedLocalSearch ils;
    SolveResult res = ils.solve(inst, cfg);

    EXPECT_TRUE(res.feasible);
    expectCheckerOk(inst, res.best);
    expectEveryCustomerOnce(inst, res.best);
    EXPECT_TRUE(!Solution::lexBetter(greedy.numVehicles(), greedy.distance(inst),
                                     res.best.numVehicles(), res.best.distance(inst)));
    EXPECT_FALSE(res.trace.empty());
}

INSTANTIATE_TEST_SUITE_P(Instances, IlsTest, ::testing::Values(std::string("c101C10"), std::string("r101_21")));
