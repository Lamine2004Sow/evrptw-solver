#pragma once

#include <string>
#include <vector>

namespace evrptw {

enum class NodeType { Depot, Station, Customer };

struct Node {
    std::string id;
    NodeType type;
    double x, y;
    double demand;
    double ready;    // earliest service start
    double due;      // latest service start
    double service;  // service duration
};

// An E-VRPTW instance in the format of Schneider, Stenger & Goeke (2014).
//
// Node indexing: 0 is the depot; then all recharging stations (in file order,
// including the station "S0" co-located with the depot); then all customers.
// Distances are unrounded Euclidean.
class Instance {
public:
    static Instance load(const std::string& path);

    const std::string& name() const { return name_; }
    int numNodes() const { return static_cast<int>(nodes_.size()); }
    int numStations() const { return numStations_; }
    int numCustomers() const { return numCustomers_; }

    int depot() const { return 0; }
    int firstStation() const { return 1; }
    int firstCustomer() const { return 1 + numStations_; }
    bool isStation(int i) const { return i >= 1 && i < firstCustomer(); }
    bool isCustomer(int i) const { return i >= firstCustomer(); }

    const Node& node(int i) const { return nodes_[i]; }
    double dist(int i, int j) const { return dist_[i * n_ + j]; }
    double travelTime(int i, int j) const { return dist(i, j) / velocity_; }
    double energy(int i, int j) const { return dist(i, j) * consumption_; }

    double batteryCapacity() const { return batteryCapacity_; }    // Q
    double loadCapacity() const { return loadCapacity_; }          // C
    double consumptionRate() const { return consumption_; }        // r
    double rechargeRate() const { return rechargeRate_; }          // g (time per energy unit)
    double velocity() const { return velocity_; }                  // v
    double horizon() const { return nodes_[0].due; }

    // Station index closest to the arc (i, j) in the detour sense, ignoring feasibility.
    int bestStationBetween(int i, int j) const;

private:
    std::string name_;
    std::vector<Node> nodes_;
    std::vector<double> dist_;
    int n_ = 0;
    int numStations_ = 0;
    int numCustomers_ = 0;
    double batteryCapacity_ = 0, loadCapacity_ = 0, consumption_ = 1, rechargeRate_ = 1, velocity_ = 1;
};

}  // namespace evrptw
