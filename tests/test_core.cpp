#include <gtest/gtest.h>

#include <cmath>
#include <fstream>

#include "evrptw/Checker.hpp"
#include "evrptw/Instance.hpp"
#include "evrptw/Solution.hpp"

using namespace evrptw;

namespace {

// Hand-built instance: depot at origin, one station, three customers on a line.
// Q=40, C=10, r=1, g=2, v=1, horizon 200.
std::string writeToyInstance() {
    std::string path = ::testing::TempDir() + "toy.txt";
    std::ofstream out(path);
    out << "StringID   Type       x          y          demand     ReadyTime  DueDate    ServiceTime\n"
           "D0 d 0 0 0 0 200 0\n"
           "S0 f 0 0 0 0 200 0\n"
           "S1 f 20 0 0 0 200 0\n"
           "C1 c 10 0 4 0 50 5\n"
           "C2 c 20 0 4 0 100 5\n"
           "C3 c 30 0 4 60 120 5\n"
           "\n"
           "Q Vehicle fuel tank capacity /40.0/\n"
           "C Vehicle load capacity /10.0/\n"
           "r fuel consumption rate /1.0/\n"
           "g inverse refueling rate /2.0/\n"
           "v average Velocity /1.0/\n";
    return path;
}

class ToyTest : public ::testing::Test {
protected:
    void SetUp() override { inst = Instance::load(writeToyInstance()); }
    int id(const std::string& s) const {
        for (int i = 0; i < inst.numNodes(); ++i)
            if (inst.node(i).id == s) return i;
        return -1;
    }
    Instance inst;
};

}  // namespace

TEST_F(ToyTest, ParsesIndexingAndParameters) {
    EXPECT_EQ(inst.numStations(), 2);
    EXPECT_EQ(inst.numCustomers(), 3);
    EXPECT_EQ(inst.node(0).type, NodeType::Depot);
    EXPECT_TRUE(inst.isStation(id("S1")));
    EXPECT_TRUE(inst.isCustomer(id("C3")));
    EXPECT_DOUBLE_EQ(inst.batteryCapacity(), 40.0);
    EXPECT_DOUBLE_EQ(inst.rechargeRate(), 2.0);
    EXPECT_DOUBLE_EQ(inst.dist(id("C1"), id("C3")), 20.0);
}

TEST_F(ToyTest, FeasibleRouteWithRecharge) {
    // Two routes (capacity 10). Route 2 must recharge at S1 to reach C3 and come back.
    Solution s;
    s.routes = {{id("C1"), id("C2")}, {id("S1"), id("C3")}};
    // route 2: 0->S1 (battery 20, recharge 40 time units -> t=60), S1->C3 (t=70 ok), C3->D0 (battery 0).
    CheckReport rep = checkSolution(inst, s);
    EXPECT_TRUE(rep.ok) << (rep.errors.empty() ? "" : rep.errors[0]);
    EXPECT_EQ(rep.vehicles, 2);
    EXPECT_NEAR(rep.distance, 40.0 + 60.0, 1e-9);
    EXPECT_TRUE(s.feasible(inst));
    EXPECT_NEAR(s.distance(inst), rep.distance, 1e-9);
}

TEST_F(ToyTest, DetectsBatteryViolation) {
    Solution s;
    s.routes = {{id("C1"), id("C2")}, {id("C3")}};  // 0->C3->0 needs 60 > Q=40
    CheckReport rep = checkSolution(inst, s);
    EXPECT_FALSE(rep.ok);
    EXPECT_GT(evaluateRoute(inst, s.routes[1]).batteryDeficit, 0.0);
}

TEST_F(ToyTest, DetectsCapacityViolation) {
    Solution s;
    s.routes = {{id("C1"), id("C2"), id("S1"), id("C3")}};  // load 12 > 10
    CheckReport rep = checkSolution(inst, s);
    EXPECT_FALSE(rep.ok);
    EXPECT_NEAR(evaluateRoute(inst, s.routes[0]).capacityExcess, 2.0, 1e-9);
}

TEST_F(ToyTest, DetectsTimeWindowViolation) {
    Solution s;
    // C3 is served at t=70, so C1 (due 50) is reached too late.
    s.routes = {{id("S1"), id("C3"), id("C1")}, {id("C2")}};
    CheckReport rep = checkSolution(inst, s);
    EXPECT_FALSE(rep.ok);
    EXPECT_GT(evaluateRoute(inst, s.routes[0]).timeWarp, 0.0);
}

TEST_F(ToyTest, DetectsMissingAndDuplicateCustomers) {
    Solution s;
    s.routes = {{id("C1"), id("C1")}, {id("C2")}};
    CheckReport rep = checkSolution(inst, s);
    EXPECT_FALSE(rep.ok);
    EXPECT_GE(rep.errors.size(), 2u);  // C1 twice, C3 never
}

TEST_F(ToyTest, SolutionRoundTrip) {
    Solution s;
    s.routes = {{id("C1"), id("C2")}, {id("S1"), id("C3")}};
    std::string path = ::testing::TempDir() + "toy_sol.txt";
    s.write(inst, path);
    Solution r = Solution::read(inst, path);
    EXPECT_EQ(r.routes, s.routes);
}

TEST(Lexicographic, VehiclesFirst) {
    EXPECT_TRUE(Solution::lexBetter(3, 900.0, 4, 500.0));
    EXPECT_FALSE(Solution::lexBetter(4, 500.0, 3, 900.0));
    EXPECT_TRUE(Solution::lexBetter(3, 500.0, 3, 501.0));
}

TEST(RealInstances, LoadsSchneiderFiles) {
    Instance small = Instance::load(std::string(EVRPTW_DATA_DIR) + "/c101C5.txt");
    EXPECT_EQ(small.numCustomers(), 5);
    Instance large = Instance::load(std::string(EVRPTW_DATA_DIR) + "/r101_21.txt");
    EXPECT_EQ(large.numCustomers(), 100);
    EXPECT_EQ(large.numStations(), 21);  // S0 (at the depot) + 20 others
}
