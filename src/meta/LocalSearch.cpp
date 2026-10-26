#include "evrptw/meta/LocalSearch.hpp"

#include <algorithm>
#include <cstdlib>
#include <iterator>
#include <limits>

#include "evrptw/meta/Constructive.hpp"

namespace evrptw {

namespace {

constexpr double kEps = 1e-6;
constexpr int kRepairStations = 3;  // stations tried per slot by the station-aware evaluation
constexpr int kMaxEliminationAttempts = 4;  // penalties 10, 100, 1000, 10000 (times start)

}  // namespace

LocalSearch::LocalSearch(const Instance& inst)
    : stationAware_(inst.numCustomers() <= kStationAwareMaxCustomers), inst_(inst) {
    const int n = inst.numNodes();
    neighbours_.assign(n, {});
    stationNeighbours_.assign(n, {});
    for (int u = inst.firstCustomer(); u < n; ++u) customers_.push_back(u);

    std::vector<int> others;
    for (int i = 1; i < n; ++i) others.push_back(i);  // every node except the depot
    for (int i = 0; i < n; ++i) {
        std::vector<int> cand;
        for (int j : others)
            if (j != i) cand.push_back(j);
        auto byDistance = [&](int a, int b) { return inst.dist(i, a) < inst.dist(i, b); };
        std::vector<int> stations;
        for (int j : cand)
            if (inst.isStation(j)) stations.push_back(j);
        std::sort(stations.begin(), stations.end(), byDistance);
        if (static_cast<int>(stations.size()) > kGranularity / 2) stations.resize(kGranularity / 2);
        stationNeighbours_[i] = stations;

        if (inst.isCustomer(i)) {
            std::sort(cand.begin(), cand.end(), byDistance);
            if (static_cast<int>(cand.size()) > kGranularity) cand.resize(kGranularity);
            neighbours_[i] = cand;
        }
    }
}

// ---------------------------------------------------------------- state

void LocalSearch::load(const Solution& sol) {
    routes_ = sol.routes;
    const int n = inst_.numNodes();
    routeOf_.assign(n, -1);
    posOf_.assign(n, -1);
    evals_.assign(routes_.size(), RouteEval{});
    penalty_.assign(routes_.size(), 0.0);
    numNonEmpty_ = 0;
    for (int r = 0; r < static_cast<int>(routes_.size()); ++r) {
        const bool hasCustomer = std::any_of(routes_[r].begin(), routes_[r].end(),
                                             [&](int j) { return inst_.isCustomer(j); });
        if (!hasCustomer) routes_[r].clear();  // station-only routes are dropped
        if (!routes_[r].empty()) ++numNonEmpty_;
        refreshRoute(r);
    }
}

void LocalSearch::store(Solution& sol) const {
    sol.routes = routes_;
    sol.removeEmptyRoutes();
}

void LocalSearch::refreshRoute(int r) {
    const Route& route = routes_[r];
    evals_[r] = evaluateRoute(inst_, route);
    penalty_[r] = penalisedCost(evals_[r], w_);
    for (int i = 0; i < static_cast<int>(route.size()); ++i)
        if (inst_.isCustomer(route[i])) {
            routeOf_[route[i]] = r;
            posOf_[route[i]] = i;
        }
}

// Replaces route r by `replacement` (swapped, so the old buffer is reused).
// Routes without customers are cleared.
void LocalSearch::setRoute(int r, Route& replacement) {
    int pos, station;  // the station detour that costOf() priced in, if any
    repairedCost(replacement, &pos, &station);
    if (pos >= 0) replacement.insert(replacement.begin() + pos, station);
    const bool wasNonEmpty = !routes_[r].empty();
    routes_[r].swap(replacement);
    const bool hasCustomer = std::any_of(routes_[r].begin(), routes_[r].end(),
                                         [&](int j) { return inst_.isCustomer(j); });
    if (!hasCustomer) routes_[r].clear();
    numNonEmpty_ += static_cast<int>(!routes_[r].empty()) - static_cast<int>(wasNonEmpty);
    refreshRoute(r);
    for (int j : routes_[r])  // customers of a changed route must be looked at again
        if (inst_.isCustomer(j)) active_[j] = 1;
}

// Customer moves often only pay off if a charger detour moves with them (optimal
// small-instance routes look like "S0 C98 S0"). Without this, such moves look
// battery-infeasible and are rejected: the ILS missed the optimum of 5-customer
// instances by up to 18%.
double LocalSearch::repairedCost(const Route& route, int* pos, int* station) const {
    const RouteEval e = evaluateRoute(inst_, route);
    double best = penalisedCost(e, w_);
    if (pos) *pos = -1;
    if (!stationAware_ || e.batteryDeficit <= kEps) return best;

    int bestPos = -1, bestStation = -1;
    repairBuf_ = route;
    for (int p = 0; p <= static_cast<int>(route.size()); ++p) {
        const int prev = p == 0 ? inst_.depot() : route[p - 1];
        const int next = p == static_cast<int>(route.size()) ? inst_.depot() : route[p];
        const auto& near = stationNeighbours_[prev];
        for (int k = 0; k < std::min<int>(kRepairStations, near.size()); ++k) {
            const int s = near[k];
            if (s == prev || s == next) continue;
            repairBuf_.insert(repairBuf_.begin() + p, s);
            const double c = penalisedCost(evaluateRoute(inst_, repairBuf_), w_);
            repairBuf_.erase(repairBuf_.begin() + p);
            if (c < best - kEps) {
                best = c;
                bestPos = p;
                bestStation = s;
            }
        }
    }
    if (pos) *pos = bestPos;
    if (station) *station = bestStation;
    return best;
}

bool LocalSearch::mayOpenRoute() const { return maxVehicles_ < 0 || numNonEmpty_ < maxVehicles_; }

// Index of an empty route, appending one if needed. -1 if none can be opened.
int LocalSearch::emptyRouteSlot() {
    for (int r = 0; r < static_cast<int>(routes_.size()); ++r)
        if (routes_[r].empty()) return r;
    routes_.emplace_back();
    evals_.emplace_back();
    penalty_.emplace_back(0.0);
    return static_cast<int>(routes_.size()) - 1;
}

// ---------------------------------------------------------------- driver

void LocalSearch::run(Solution& sol, const PenaltyWeights& w, Rng& rng, int maxVehicles) {
    descend(sol, w, rng, maxVehicles, nullptr);
}

void LocalSearch::runFrom(Solution& sol, const PenaltyWeights& w, Rng& rng, int maxVehicles,
                          const std::vector<int>& touched) {
    descend(sol, w, rng, maxVehicles, &touched);
}

void LocalSearch::descend(Solution& sol, const PenaltyWeights& w, Rng& rng, int maxVehicles,
                          const std::vector<int>* touched) {
    w_ = w;
    rng_ = &rng;
    maxVehicles_ = maxVehicles;
    load(sol);

    if (touched == nullptr) {
        active_.assign(inst_.numNodes(), 1);
    } else {
        active_.assign(inst_.numNodes(), 0);
        for (int c : *touched) {
            if (!inst_.isCustomer(c) || routeOf_[c] < 0) continue;
            for (int j : routes_[routeOf_[c]])
                if (inst_.isCustomer(j)) active_[j] = 1;
        }
    }
    std::vector<int> order = customers_;
    bool improved = true;
    while (improved) {
        improved = false;
        std::shuffle(order.begin(), order.end(), rng);
        for (int u : order) {
            if (!active_[u]) continue;
            if (improveCustomer(u)) improved = true;
            else active_[u] = 0;
        }
        while (stationRemoval() || stationInsertion()) improved = true;
    }
    store(sol);
    rng_ = nullptr;
}

bool LocalSearch::improveCustomer(int u) {
    using Move = bool (LocalSearch::*)(int);
    Move moves[] = {&LocalSearch::relocate, &LocalSearch::swapCustomers, &LocalSearch::twoOptStar,
                    &LocalSearch::twoOpt};
    std::shuffle(std::begin(moves), std::end(moves), *rng_);
    for (Move m : moves)
        if ((this->*m)(u)) return true;
    return false;
}

// ---------------------------------------------------------------- customer moves

bool LocalSearch::relocate(int u) {
    const int ru = routeOf_[u];
    const int pu = posOf_[u];

    // Route of u without u: the same for every target, so it is evaluated once.
    Route without = routes_[ru];
    without.erase(without.begin() + pu);
    const double costWithout = costOf(without);

    for (int v : neighbours_[u]) {
        if (!inst_.isCustomer(v)) continue;
        const int rv = routeOf_[v];
        const int pv = posOf_[v];
        if (rv == ru) {
            bufA_ = without;
            const int slot = pv > pu ? pv - 1 : pv;  // index of v once u is removed
            for (int at : {slot, slot + 1}) {
                bufA_.insert(bufA_.begin() + at, u);
                if (costOf(bufA_) < penalty_[ru] - kEps) {
                    setRoute(ru, bufA_);
                    return true;
                }
                bufA_.erase(bufA_.begin() + at);
            }
        } else {
            for (int at : {pv, pv + 1}) {
                bufB_ = routes_[rv];
                bufB_.insert(bufB_.begin() + at, u);
                if (costWithout + costOf(bufB_) < penalty_[ru] + penalty_[rv] - kEps) {
                    bufA_ = without;
                    setRoute(ru, bufA_);
                    setRoute(rv, bufB_);
                    return true;
                }
            }
        }
    }

    // Move u alone into an empty route, if that is cheaper and allowed.
    if (routes_[ru].size() > 1 && mayOpenRoute()) {
        const int e = emptyRouteSlot();
        bufB_.assign(1, u);
        if (costWithout + costOf(bufB_) < penalty_[ru] - kEps) {
            bufA_ = without;
            setRoute(ru, bufA_);
            setRoute(e, bufB_);
            return true;
        }
    }
    return false;
}

bool LocalSearch::swapCustomers(int u) {
    const int ru = routeOf_[u];
    const int pu = posOf_[u];
    for (int v : neighbours_[u]) {
        if (!inst_.isCustomer(v)) continue;
        const int rv = routeOf_[v];
        const int pv = posOf_[v];
        if (rv == ru) {
            if (pu == pv) continue;
            bufA_ = routes_[ru];
            std::swap(bufA_[pu], bufA_[pv]);
            if (costOf(bufA_) < penalty_[ru] - kEps) {
                setRoute(ru, bufA_);
                return true;
            }
        } else {
            bufA_ = routes_[ru];
            bufB_ = routes_[rv];
            bufA_[pu] = v;
            bufB_[pv] = u;
            if (costOf(bufA_) + costOf(bufB_) < penalty_[ru] + penalty_[rv] - kEps) {
                setRoute(ru, bufA_);
                setRoute(rv, bufB_);
                return true;
            }
        }
    }
    return false;
}

// 2-opt*: exchange route tails between two routes. Two variants are tried:
//  - tails swapped:  A[..pu] + B[pv+1..]  and  B[..pv] + A[pu+1..]
//  - tails reversed: A[..pu] + rev(B[..pv]) and rev(A[pu+1..]) + B[pv+1..]  (adds edge u-v)
bool LocalSearch::twoOptStar(int u) {
    const int ru = routeOf_[u];
    const int pu = posOf_[u];
    for (int v : neighbours_[u]) {
        if (!inst_.isCustomer(v)) continue;
        const int rv = routeOf_[v];
        if (rv == ru) continue;
        const int pv = posOf_[v];
        const Route& A = routes_[ru];
        const Route& B = routes_[rv];

        bufA_.assign(A.begin(), A.begin() + pu + 1);
        bufA_.insert(bufA_.end(), B.begin() + pv + 1, B.end());
        bufB_.assign(B.begin(), B.begin() + pv + 1);
        bufB_.insert(bufB_.end(), A.begin() + pu + 1, A.end());
        if (costOf(bufA_) + costOf(bufB_) < penalty_[ru] + penalty_[rv] - kEps) {
            setRoute(ru, bufA_);
            setRoute(rv, bufB_);
            return true;
        }

        bufA_.assign(A.begin(), A.begin() + pu + 1);
        for (int i = pv; i >= 0; --i) bufA_.push_back(B[i]);
        bufB_.clear();
        for (int i = static_cast<int>(A.size()) - 1; i > pu; --i) bufB_.push_back(A[i]);
        bufB_.insert(bufB_.end(), B.begin() + pv + 1, B.end());
        if (costOf(bufA_) + costOf(bufB_) < penalty_[ru] + penalty_[rv] - kEps) {
            setRoute(ru, bufA_);
            setRoute(rv, bufB_);
            return true;
        }
    }
    return false;
}

// Intra-route 2-opt: reverse the segment strictly between u and a granular neighbour.
bool LocalSearch::twoOpt(int u) {
    const int ru = routeOf_[u];
    const int pu = posOf_[u];
    for (int v : neighbours_[u]) {
        if (!inst_.isCustomer(v) || routeOf_[v] != ru) continue;
        const int pv = posOf_[v];
        if (std::abs(pu - pv) < 2) continue;
        const int i = std::min(pu, pv);
        const int j = std::max(pu, pv);
        bufA_ = routes_[ru];
        std::reverse(bufA_.begin() + i + 1, bufA_.begin() + j + 1);
        if (costOf(bufA_) < penalty_[ru] - kEps) {
            setRoute(ru, bufA_);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------- station moves

bool LocalSearch::stationRemoval() {
    for (int r = 0; r < static_cast<int>(routes_.size()); ++r) {
        const Route& route = routes_[r];
        for (int k = 0; k < static_cast<int>(route.size()); ++k) {
            if (!inst_.isStation(route[k])) continue;
            bufA_ = route;
            bufA_.erase(bufA_.begin() + k);
            if (costOf(bufA_) < penalty_[r] - kEps) {
                setRoute(r, bufA_);
                return true;
            }
        }
    }
    return false;
}

// Inserts a station next to a node of a route that violates the battery constraint.
bool LocalSearch::stationInsertion() {
    for (int r = 0; r < static_cast<int>(routes_.size()); ++r) {
        if (evals_[r].batteryDeficit <= kEps) continue;
        bufA_ = routes_[r];
        for (int p = 0; p <= static_cast<int>(bufA_.size()); ++p) {
            const int prev = p == 0 ? inst_.depot() : bufA_[p - 1];
            for (int s : stationNeighbours_[prev]) {
                bufA_.insert(bufA_.begin() + p, s);
                if (costOf(bufA_) < penalty_[r] - kEps) {
                    setRoute(r, bufA_);
                    return true;
                }
                bufA_.erase(bufA_.begin() + p);
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------- route elimination

bool LocalSearch::tryEliminateRoute(Solution& sol, Rng& rng, const Timer& timer, double deadline,
                                    const PenaltyWeights& start) {
    if (sol.numVehicles() <= 1) return false;
    Solution trial = sol;
    if (!dissolveRoute(inst_, trial, shortestRouteIndex(inst_, sol), start, rng)) return false;

    const int target = sol.numVehicles() - 1;
    PenaltyWeights w = start;
    for (int attempt = 0; attempt < kMaxEliminationAttempts; ++attempt) {
        if (timer.elapsed() > deadline) break;
        run(trial, w, rng, target);
        if (trial.feasible(inst_) && trial.numVehicles() <= target) {
            trial.removeEmptyRoutes();
            sol = std::move(trial);
            return true;
        }
        w.capacity *= 10;
        w.time *= 10;
        w.battery *= 10;
    }
    return false;
}

}  // namespace evrptw
