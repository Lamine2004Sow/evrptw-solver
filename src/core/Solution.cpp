#include "evrptw/Solution.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace evrptw {

RouteEval evaluateRoute(const Instance& inst, const Route& route) {
    RouteEval e;
    const double Q = inst.batteryCapacity();
    double time = inst.node(0).ready;
    double battery = Q;
    int prev = inst.depot();

    auto arrive = [&](int j) {
        e.distance += inst.dist(prev, j);
        time += inst.travelTime(prev, j);
        battery -= inst.energy(prev, j);
        if (battery < 0) {
            e.batteryDeficit += -battery;
            battery = 0;
        }
        const Node& n = inst.node(j);
        time = std::max(time, n.ready);
        if (time > n.due) {
            e.timeWarp += time - n.due;
            time = n.due;
        }
        prev = j;
    };

    for (int j : route) {
        arrive(j);
        if (inst.isStation(j)) {
            time += inst.rechargeRate() * (Q - battery);
            battery = Q;
        } else {
            e.load += inst.node(j).demand;
            time += inst.node(j).service;
        }
    }
    arrive(inst.depot());
    e.duration = time;
    e.capacityExcess = std::max(0.0, e.load - inst.loadCapacity());
    return e;
}

int Solution::numVehicles() const {
    return static_cast<int>(std::count_if(routes.begin(), routes.end(),
                                          [](const Route& r) { return !r.empty(); }));
}

double Solution::distance(const Instance& inst) const {
    double d = 0;
    for (const auto& r : routes)
        if (!r.empty()) d += evaluateRoute(inst, r).distance;
    return d;
}

double Solution::penalizedCost(const Instance& inst, const PenaltyWeights& w) const {
    double c = 0;
    for (const auto& r : routes) {
        if (r.empty()) continue;
        RouteEval e = evaluateRoute(inst, r);
        c += e.distance + w.capacity * e.capacityExcess + w.time * e.timeWarp +
             w.battery * e.batteryDeficit;
    }
    return c;
}

bool Solution::feasible(const Instance& inst) const {
    for (const auto& r : routes)
        if (!r.empty() && !evaluateRoute(inst, r).feasible()) return false;
    return true;
}

void Solution::removeEmptyRoutes() {
    routes.erase(std::remove_if(routes.begin(), routes.end(), [](const Route& r) { return r.empty(); }),
                 routes.end());
}

bool Solution::lexBetter(int vehA, double distA, int vehB, double distB, double eps) {
    if (vehA != vehB) return vehA < vehB;
    return distA < distB - eps;
}

std::string Solution::toString(const Instance& inst) const {
    std::ostringstream out;
    const std::string& d = inst.node(0).id;
    for (const auto& r : routes) {
        if (r.empty()) continue;
        out << d;
        for (int j : r) out << ' ' << inst.node(j).id;
        out << ' ' << d << '\n';
    }
    return out.str();
}

void Solution::write(const Instance& inst, const std::string& path) const {
    std::ofstream out(path);
    if (!out) throw std::runtime_error("cannot write solution: " + path);
    out.precision(10);
    out << "# instance " << inst.name() << '\n';
    out << "# vehicles " << numVehicles() << '\n';
    out << "# distance " << distance(inst) << '\n';
    out << toString(inst);
}

Solution Solution::read(const Instance& inst, const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open solution: " + path);
    std::unordered_map<std::string, int> index;
    for (int i = 0; i < inst.numNodes(); ++i) index[inst.node(i).id] = i;

    Solution s;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string tok;
        Route r;
        while (ss >> tok) {
            auto it = index.find(tok);
            if (it == index.end()) throw std::runtime_error("unknown node id in solution: " + tok);
            if (it->second != inst.depot()) r.push_back(it->second);
        }
        if (!r.empty()) s.routes.push_back(r);
    }
    return s;
}

}  // namespace evrptw
