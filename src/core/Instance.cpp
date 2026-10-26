#include "evrptw/Instance.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace evrptw {

namespace {

// Parses the value of a parameter line such as "Q Vehicle fuel tank capacity /77.75/".
double parseSlashValue(const std::string& line) {
    auto a = line.find('/');
    auto b = line.rfind('/');
    if (a == std::string::npos || b == a) throw std::runtime_error("bad parameter line: " + line);
    return std::stod(line.substr(a + 1, b - a - 1));
}

std::string baseName(const std::string& path) {
    auto slash = path.find_last_of("/\\");
    std::string f = slash == std::string::npos ? path : path.substr(slash + 1);
    auto dot = f.rfind('.');
    return dot == std::string::npos ? f : f.substr(0, dot);
}

}  // namespace

Instance Instance::load(const std::string& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open instance: " + path);

    Instance inst;
    inst.name_ = baseName(path);

    std::vector<Node> depots, stations, customers;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        if (line.find_first_not_of(" \t\r") == std::string::npos) continue;
        if (line.find('/') != std::string::npos) {
            double v = parseSlashValue(line);
            switch (line[0]) {
                case 'Q': inst.batteryCapacity_ = v; break;
                case 'C': inst.loadCapacity_ = v; break;
                case 'r': inst.consumption_ = v; break;
                case 'g': inst.rechargeRate_ = v; break;
                case 'v': inst.velocity_ = v; break;
                default: throw std::runtime_error("unknown parameter: " + line);
            }
            continue;
        }
        std::istringstream ss(line);
        Node n;
        std::string type;
        ss >> n.id >> type >> n.x >> n.y >> n.demand >> n.ready >> n.due >> n.service;
        if (!ss) throw std::runtime_error("bad node line: " + line);
        if (type == "d") { n.type = NodeType::Depot; depots.push_back(n); }
        else if (type == "f") { n.type = NodeType::Station; stations.push_back(n); }
        else if (type == "c") { n.type = NodeType::Customer; customers.push_back(n); }
        else throw std::runtime_error("unknown node type: " + type);
    }
    if (depots.size() != 1) throw std::runtime_error("instance must have exactly one depot");

    inst.nodes_.push_back(depots[0]);
    inst.nodes_.insert(inst.nodes_.end(), stations.begin(), stations.end());
    inst.nodes_.insert(inst.nodes_.end(), customers.begin(), customers.end());
    inst.numStations_ = static_cast<int>(stations.size());
    inst.numCustomers_ = static_cast<int>(customers.size());
    inst.n_ = inst.numNodes();

    inst.dist_.assign(inst.n_ * inst.n_, 0.0);
    for (int i = 0; i < inst.n_; ++i)
        for (int j = 0; j < inst.n_; ++j)
            inst.dist_[i * inst.n_ + j] = std::hypot(inst.nodes_[i].x - inst.nodes_[j].x,
                                                     inst.nodes_[i].y - inst.nodes_[j].y);
    return inst;
}

int Instance::bestStationBetween(int i, int j) const {
    int best = -1;
    double bestCost = std::numeric_limits<double>::infinity();
    for (int s = firstStation(); s < firstCustomer(); ++s) {
        if (s == i || s == j) continue;
        double c = dist(i, s) + dist(s, j);
        if (c < bestCost) { bestCost = c; best = s; }
    }
    return best;
}

}  // namespace evrptw
