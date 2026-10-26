#include "evrptw/meta/ILS.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <vector>

#include "evrptw/meta/Constructive.hpp"
#include "evrptw/meta/LocalSearch.hpp"

namespace evrptw {

namespace {

constexpr double kPhaseAShare = 0.4;          // share of the time budget for fleet minimisation
constexpr double kAcceptShare = 0.01;         // initial distance slack for equal-fleet acceptance
constexpr double kMaxPenalty = 1e4;
constexpr int kAdaptPeriod = 100;             // iterations between penalty updates
constexpr int kRetryEliminateEvery = 100;     // phase B iterations between elimination attempts
constexpr int kFleetSearchIterations = 2000;  // budget of a fixed-fleet search in phase A
constexpr int kFleetSearchIterationsB = 500;  // budget of one fixed-fleet attempt in phase B
constexpr int kFleetRetryEvery = 1000;        // phase B iterations between fixed-fleet attempts
constexpr int kMaxFleetGap = 8000;            // back-off cap for fixed-fleet attempts

// Removes a set of customers and reinserts them by cheapest penalised insertion.
class RuinRecreate {
public:
    RuinRecreate(const Instance& inst, const LocalSearch& ls) : inst_(inst), ls_(ls) {}

    // Returns the perturbed solution; `touched` receives the removed customers and
    // their granular neighbours, the only starting points of the local search.
    Solution perturb(const Solution& current, const PenaltyWeights& w, Rng& rng,
                     std::vector<int>& touched) const {
        Solution s = current;
        const int n = inst_.numCustomers();
        const int lo = std::max(1, n / 10), hi = std::max(1, n / 5);
        const int k = std::uniform_int_distribution<int>(lo, hi)(rng);
        const std::vector<int> removed = chooseCustomers(k, rng);
        touched = removed;
        for (int c : removed)
            for (int v : ls_.neighbours(c))
                if (inst_.isCustomer(v)) touched.push_back(v);

        std::vector<char> marked(inst_.numNodes(), 0);
        for (int c : removed) marked[c] = 1;
        for (Route& r : s.routes)
            r.erase(std::remove_if(r.begin(), r.end(), [&](int j) { return marked[j] != 0; }), r.end());
        s.removeEmptyRoutes();

        std::vector<int> order = removed;
        std::shuffle(order.begin(), order.end(), rng);
        Route scratch;
        for (int c : order) {
            const InsertionMove m = bestInsertion(inst_, s.routes, c, w, false, scratch);
            if (m.route < 0) {
                s.routes.push_back(Route{c});
            } else {
                s.routes[m.route].insert(s.routes[m.route].begin() + m.position, c);
            }
        }
        return s;
    }

private:
    // Half of the time: random customers. Otherwise a BFS on the granular graph
    // from a random seed, so that the removed customers are geographically close.
    std::vector<int> chooseCustomers(int k, Rng& rng) const {
        std::vector<int> all;
        for (int c = inst_.firstCustomer(); c < inst_.numNodes(); ++c) all.push_back(c);
        std::shuffle(all.begin(), all.end(), rng);

        if (std::bernoulli_distribution(0.5)(rng)) {
            all.resize(k);
            return all;
        }
        std::vector<char> chosen(inst_.numNodes(), 0);
        std::vector<int> queue{all.front()};
        chosen[all.front()] = 1;
        for (std::size_t head = 0; head < queue.size() && static_cast<int>(queue.size()) < k; ++head) {
            for (int v : ls_.neighbours(queue[head])) {
                if (!inst_.isCustomer(v) || chosen[v]) continue;
                chosen[v] = 1;
                queue.push_back(v);
                if (static_cast<int>(queue.size()) >= k) break;
            }
        }
        for (int c : all) {  // fill up if the granular graph ran out
            if (static_cast<int>(queue.size()) >= k) break;
            if (!chosen[c]) {
                chosen[c] = 1;
                queue.push_back(c);
            }
        }
        return queue;
    }

    const Instance& inst_;
    const LocalSearch& ls_;
};

// Multiplies the penalty weights by 1.2 when few local search outcomes are feasible
// and by 0.85 when most are, every kAdaptPeriod outcomes.
class AdaptivePenalties {
public:
    const PenaltyWeights& weights() const { return w_; }

    void record(bool feasible) {
        ++count_;
        if (feasible) ++feasibleCount_;
        if (count_ < kAdaptPeriod) return;
        const double ratio = static_cast<double>(feasibleCount_) / count_;
        const double factor = ratio < 0.25 ? 1.2 : (ratio > 0.75 ? 0.85 : 1.0);
        w_.capacity = clamp(w_.capacity * factor);
        w_.time = clamp(w_.time * factor);
        w_.battery = clamp(w_.battery * factor);
        count_ = feasibleCount_ = 0;
    }

private:
    static double clamp(double x) { return std::min(kMaxPenalty, std::max(1.0, x)); }

    PenaltyWeights w_;
    int count_ = 0;
    int feasibleCount_ = 0;
};

// Fixed-fleet search for one route less: the shortest route is dissolved, then ruin &
// recreate with local search minimises the penalised cost at the reduced fleet size
// until a feasible solution appears. Returns true and updates sol on success.
bool reduceFleet(const Instance& inst, LocalSearch& ls, const RuinRecreate& ruinRecreate, Solution& sol,
                 Rng& rng, const Timer& timer, double deadline, int budget) {
    const int target = sol.numVehicles() - 1;
    AdaptivePenalties penalties;
    Solution trial = sol;
    if (!dissolveRoute(inst, trial, shortestRouteIndex(inst, sol), penalties.weights(), rng)) return false;
    ls.run(trial, penalties.weights(), rng, target);
    std::vector<int> touched;
    for (int it = 0; it < budget && timer.elapsed() < deadline; ++it) {
        if (trial.feasible(inst)) {
            sol = trial;
            sol.removeEmptyRoutes();
            return true;
        }
        Solution cand = ruinRecreate.perturb(trial, penalties.weights(), rng, touched);
        ls.runFrom(cand, penalties.weights(), rng, target, touched);
        penalties.record(cand.feasible(inst));
        if (cand.penalizedCost(inst, penalties.weights()) <= trial.penalizedCost(inst, penalties.weights()))
            trial = std::move(cand);
    }
    return false;
}

// Best feasible solution (within the fleet cap) seen so far, with its trace.
class Incumbent {
public:
    Incumbent(const Instance& inst, int maxVehicles, const Timer& timer, SolveResult& result)
        : inst_(inst), maxVehicles_(maxVehicles), timer_(timer), result_(result) {}

    bool fleetOk(int vehicles) const { return maxVehicles_ <= 0 || vehicles <= maxVehicles_; }

    // Offers a solution; keeps it if it is feasible, within the cap and lexicographically better.
    bool offer(const Solution& sol, bool isFeasible) {
        if (!isFeasible) return false;
        const int veh = sol.numVehicles();
        if (!fleetOk(veh)) return false;
        const double dist = sol.distance(inst_);
        if (have_ && !Solution::lexBetter(veh, dist, best_.numVehicles(), bestDist_)) return false;
        best_ = sol;
        bestDist_ = dist;
        have_ = true;
        result_.trace.push_back({timer_.elapsed(), veh, dist});
        if (verbose_) std::cerr << "[ils] t=" << timer_.elapsed() << " vehicles=" << veh << " distance=" << dist << '\n';
        return true;
    }

    bool have() const { return have_; }
    const Solution& solution() const { return best_; }
    void setVerbose(bool v) { verbose_ = v; }

private:
    const Instance& inst_;
    int maxVehicles_;
    const Timer& timer_;
    SolveResult& result_;
    Solution best_;
    double bestDist_ = 0;
    bool have_ = false;
    bool verbose_ = false;
};

}  // namespace

SolveResult IteratedLocalSearch::solve(const Instance& inst, const SolverConfig& cfg) {
    Timer timer;
    Rng rng(cfg.seed);
    SolveResult result;
    const double limit = cfg.timeLimit;
    LocalSearch ls(inst);
    Incumbent incumbent(inst, cfg.maxVehicles, timer, result);
    incumbent.setVerbose(cfg.verbose);

    // Start from the greedy solution, polished by local search if that stays feasible.
    Solution cur = buildInitialSolution(inst, rng);
    Solution polished = cur;
    ls.run(polished, PenaltyWeights{}, rng);
    if (polished.feasible(inst) && polished.distance(inst) <= cur.distance(inst)) cur = polished;
    incumbent.offer(cur, cur.feasible(inst));

    // Phase A: fleet minimisation by route elimination, then by fixed-fleet search.
    // Stops at the phase boundary, unless the fleet cap is still unmet.
    RuinRecreate ruinRecreate(inst, ls);
    while (cur.numVehicles() > 1) {
        const bool mustReduce = cfg.maxVehicles > 0 && cur.numVehicles() > cfg.maxVehicles;
        const double deadline = mustReduce ? limit : kPhaseAShare * limit;
        if (timer.elapsed() > deadline) break;
        bool reduced = ls.tryEliminateRoute(cur, rng, timer, deadline);
        if (!reduced)
            reduced = reduceFleet(inst, ls, ruinRecreate, cur, rng, timer, deadline, kFleetSearchIterations);
        if (!reduced) {
            if (!mustReduce) break;
            continue;
        }
        incumbent.offer(cur, true);
        if (cfg.verbose) std::cerr << "[ils] phase A: vehicles=" << cur.numVehicles() << '\n';
    }

    // Phase B: distance minimisation.
    AdaptivePenalties penalties;
    int iter = 0;
    int nextFleetAttempt = kFleetRetryEvery, fleetGap = kFleetRetryEvery;
    std::vector<int> touched;
    while (timer.elapsed() < limit) {
        ++iter;
        const double frac = timer.elapsed() / limit;
        Solution cand = ruinRecreate.perturb(cur, penalties.weights(), rng, touched);
        ls.runFrom(cand, penalties.weights(), rng, cur.numVehicles(), touched);
        const bool candFeasible = cand.feasible(inst);
        penalties.record(candFeasible);

        if (candFeasible) {
            const int vc = cand.numVehicles(), vu = cur.numVehicles();
            const double dc = cand.distance(inst), du = cur.distance(inst);
            const double slack = kAcceptShare * (1.0 - frac) * du;
            if (vc < vu || (vc == vu && dc <= du + slack)) cur = cand;
            incumbent.offer(cand, true);
        }

        const bool overCap = cfg.maxVehicles > 0 && cur.numVehicles() > cfg.maxVehicles;
        if (iter % kRetryEliminateEvery == 0 || overCap) {
            Solution trial = cur;
            if (ls.tryEliminateRoute(trial, rng, timer, limit)) {
                cur = trial;
                incumbent.offer(cur, true);
            }
        }
        if (iter >= nextFleetAttempt && cur.numVehicles() > 1) {
            if (reduceFleet(inst, ls, ruinRecreate, cur, rng, timer, limit, kFleetSearchIterationsB)) {
                incumbent.offer(cur, true);
                fleetGap = kFleetRetryEvery;
            } else {
                fleetGap = std::min(fleetGap * 2, kMaxFleetGap);  // back off while fleet search fails
            }
            nextFleetAttempt = iter + fleetGap;
        }
        if (cfg.verbose && iter % 500 == 0)
            std::cerr << "[ils] t=" << timer.elapsed() << " iter=" << iter << " cur=" << cur.numVehicles()
                      << "/" << cur.distance(inst) << '\n';
    }

    if (incumbent.have()) {
        result.best = incumbent.solution();
        result.feasible = true;
    } else {
        result.best = cur;
        result.feasible = false;
    }
    result.runtime = timer.elapsed();
    return result;
}

}  // namespace evrptw
