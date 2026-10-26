#include "evrptw/Checker.hpp"

#include <cmath>
#include <sstream>

namespace evrptw {

namespace {

double euclid(const Node& a, const Node& b) { return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y)); }

}  // namespace

CheckReport checkSolution(const Instance& inst, const Solution& sol, double eps) {
    CheckReport rep;
    auto fail = [&](const std::string& msg) {
        rep.ok = false;
        rep.errors.push_back(msg);
    };

    std::vector<int> visits(inst.numNodes(), 0);
    const double Q = inst.batteryCapacity();
    const Node& depot = inst.node(inst.depot());

    for (size_t k = 0; k < sol.routes.size(); ++k) {
        const Route& r = sol.routes[k];
        if (r.empty()) continue;
        ++rep.vehicles;

        std::vector<int> path;
        path.push_back(inst.depot());
        path.insert(path.end(), r.begin(), r.end());
        path.push_back(inst.depot());

        double time = depot.ready, battery = Q, load = 0;
        for (size_t p = 1; p < path.size(); ++p) {
            int i = path[p - 1], j = path[p];
            std::ostringstream where;
            where << "route " << k << " pos " << p;
            if (j < 0 || j >= inst.numNodes()) {
                fail(where.str() + ": invalid node index");
                break;
            }
            where << " (" << inst.node(j).id << ")";
            if (p + 1 < path.size() && j == inst.depot()) fail(where.str() + ": depot inside route");

            const Node& a = inst.node(i);
            const Node& b = inst.node(j);
            double d = euclid(a, b);
            rep.distance += d;
            time += d / inst.velocity();
            battery -= d * inst.consumptionRate();
            if (battery < -eps) fail(where.str() + ": battery " + std::to_string(battery) + " < 0");

            if (time < b.ready) time = b.ready;
            if (time > b.due + eps)
                fail(where.str() + ": arrival " + std::to_string(time) + " > due " + std::to_string(b.due));

            if (b.type == NodeType::Customer) {
                ++visits[j];
                load += b.demand;
                time += b.service;
            } else if (b.type == NodeType::Station) {
                time += inst.rechargeRate() * (Q - battery);
                battery = Q;
            }
        }
        if (load > inst.loadCapacity() + eps)
            fail("route " + std::to_string(k) + ": load " + std::to_string(load) + " > capacity");
    }

    for (int c = inst.firstCustomer(); c < inst.numNodes(); ++c)
        if (visits[c] != 1)
            fail("customer " + inst.node(c).id + " visited " + std::to_string(visits[c]) + " times");

    return rep;
}

}  // namespace evrptw
