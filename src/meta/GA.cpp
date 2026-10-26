#include "evrptw/meta/GA.hpp"

#include <algorithm>
#include <iostream>
#include <limits>
#include <numeric>

#include "evrptw/meta/Constructive.hpp"
#include "evrptw/meta/ILS.hpp"
#include "evrptw/meta/LocalSearch.hpp"

namespace evrptw {

namespace {

constexpr int kMu = 25;              // population size after survivor selection
constexpr int kLambda = 40;          // offspring per generation
constexpr int kNbElite = 4;          // best individuals protected by the biased fitness
constexpr int kNbClose = 3;          // closest individuals averaged for the diversity score
constexpr int kInitialGreedy = 5;    // initial individuals built by buildInitialSolution
constexpr double kFleetShare = 0.3;  // share of the budget with a fleet attempt every generation
constexpr double kIlsSeedShare = 0.25;  // share of the budget given to the ILS that seeds the fleet size
constexpr int kFleetEvery = 50;      // generations between fleet attempts afterwards
constexpr int kAdaptPeriod = 50;     // offspring between penalty updates
constexpr double kMaxPenalty = 1e4;

// Penalised cost split by component, so that any penalty weights can be applied.
struct Costs {
    double distance = 0, capacity = 0, time = 0, battery = 0;

    double penalised(const PenaltyWeights& w) const {
        return distance + w.capacity * capacity + w.time * time + w.battery * battery;
    }
};

Costs measure(const Instance& inst, const Solution& sol) {
    Costs c;
    for (const Route& r : sol.routes) {
        if (r.empty()) continue;
        const RouteEval e = evaluateRoute(inst, r);
        c.distance += e.distance;
        c.capacity += e.capacityExcess;
        c.time += e.timeWarp;
        c.battery += e.batteryDeficit;
    }
    return c;
}

// A chromosome with its phenotype: the educated solution, its giant tour, and the
// successor/predecessor links used by the broken-pairs diversity measure.
struct Individual {
    Solution sol;
    std::vector<int> tour;
    std::vector<int> succ, pred;  // per node; the depot (0) closes each route
    Costs costs;
    bool feasible = false;
};

Individual makeIndividual(const Instance& inst, Solution sol) {
    sol.removeEmptyRoutes();
    Individual ind;
    ind.tour = Split::encode(sol, inst);
    ind.succ.assign(inst.numNodes(), inst.depot());
    ind.pred.assign(inst.numNodes(), inst.depot());
    for (const Route& r : sol.routes) {
        int prev = inst.depot();
        for (int j : r) {
            if (!inst.isCustomer(j)) continue;
            ind.pred[j] = prev;
            if (prev != inst.depot()) ind.succ[prev] = j;
            prev = j;
        }
    }
    ind.costs = measure(inst, sol);
    ind.feasible = sol.feasible(inst);
    ind.sol = std::move(sol);
    return ind;
}

// Successor links of a that are absent from b (Vidal's broken pairs, one direction).
int oneWayDifference(const Individual& a, const Individual& b) {
    int d = 0;
    for (int c : a.tour)
        if (a.succ[c] != b.succ[c] && a.succ[c] != b.pred[c]) ++d;
    return d;
}

// Symmetric broken-pairs distance: zero exactly when the two tours share their edges.
int brokenPairsDistance(const Individual& a, const Individual& b) {
    return oneWayDifference(a, b) + oneWayDifference(b, a);
}

std::vector<std::vector<int>> distanceMatrix(const std::vector<Individual>& pop, int count) {
    std::vector<std::vector<int>> d(count, std::vector<int>(count, 0));
    for (int i = 0; i < count; ++i)
        for (int j = i + 1; j < count; ++j) d[i][j] = d[j][i] = brokenPairsDistance(pop[i], pop[j]);
    return d;
}

// Normalised rank in [0, 1] (0 = best) of each key; larger keys first if `descending`.
std::vector<double> normalisedRank(const std::vector<double>& key, bool descending) {
    const int n = static_cast<int>(key.size());
    std::vector<int> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
        return descending ? key[a] > key[b] : key[a] < key[b];
    });
    std::vector<double> rank(n);
    for (int pos = 0; pos < n; ++pos) rank[order[pos]] = static_cast<double>(pos) / (n - 1);
    return rank;
}

// Biased fitness (lower is better): a cost rank plus a diversity rank, the latter
// weighted down for the elites. Diversity is the mean distance to the closest individuals.
std::vector<double> biasedFitness(const std::vector<double>& cost, const std::vector<std::vector<int>>& dist) {
    const int n = static_cast<int>(cost.size());
    if (n <= 1) return std::vector<double>(n, 0.0);
    const int k = std::min(kNbClose, n - 1);
    std::vector<double> diversity(n);
    for (int i = 0; i < n; ++i) {
        std::vector<int> d;
        for (int j = 0; j < n; ++j)
            if (j != i) d.push_back(dist[i][j]);
        std::partial_sort(d.begin(), d.begin() + k, d.end());
        diversity[i] = std::accumulate(d.begin(), d.begin() + k, 0.0) / k;
    }
    const std::vector<double> rankCost = normalisedRank(cost, false);
    const std::vector<double> rankDiv = normalisedRank(diversity, true);
    const double divWeight = std::max(0.0, 1.0 - static_cast<double>(kNbElite) / n);
    std::vector<double> fit(n);
    for (int i = 0; i < n; ++i) fit[i] = rankCost[i] + divWeight * rankDiv[i];
    return fit;
}

class Population {
public:
    void add(Individual ind) { members_.push_back(std::move(ind)); }
    int size() const { return static_cast<int>(members_.size()); }
    const Individual& at(int i) const { return members_[i]; }

    // Biased fitness of the first `count` members.
    std::vector<double> fitness(const PenaltyWeights& w, int count) const {
        std::vector<double> cost(count);
        for (int i = 0; i < count; ++i) cost[i] = members_[i].costs.penalised(w);
        return biasedFitness(cost, distanceMatrix(members_, count));
    }

    // Survivor selection: removes the worst member until `mu` remain.
    void reduceTo(int mu, const PenaltyWeights& w) {
        while (size() > mu) removeWorst(w);
    }

private:
    // Clones (zero distance to another member) go first, the worst of them; otherwise
    // the member with the highest biased fitness.
    void removeWorst(const PenaltyWeights& w) {
        const int n = size();
        std::vector<double> cost(n);
        for (int i = 0; i < n; ++i) cost[i] = members_[i].costs.penalised(w);
        const auto dist = distanceMatrix(members_, n);

        int victim = -1;
        for (int i = 0; i < n; ++i) {
            bool clone = false;
            for (int j = 0; j < n && !clone; ++j) clone = j != i && dist[i][j] == 0;
            if (clone && (victim < 0 || cost[i] > cost[victim])) victim = i;
        }
        if (victim < 0) {
            const std::vector<double> fit = biasedFitness(cost, dist);
            victim = static_cast<int>(std::max_element(fit.begin(), fit.end()) - fit.begin());
        }
        members_.erase(members_.begin() + victim);
    }

    std::vector<Individual> members_;
};

// Multiplies the penalty weights by 1.2 when few offspring are feasible and by 0.85
// when most are, aiming at about half of the offspring feasible.
class AdaptivePenalties {
public:
    const PenaltyWeights& weights() const { return w_; }

    void record(bool feasible) {
        ++count_;
        if (feasible) ++feasibleCount_;
        if (count_ < kAdaptPeriod) return;
        const double ratio = static_cast<double>(feasibleCount_) / count_;
        const double factor = ratio < 0.4 ? 1.2 : (ratio > 0.6 ? 0.85 : 1.0);
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

// Best feasible solution (within the fleet cap) seen so far, with its trace.
class BestTracker {
public:
    BestTracker(const Instance& inst, int maxVehicles, const Timer& timer, SolveResult& result)
        : inst_(inst), maxVehicles_(maxVehicles), timer_(timer), result_(result) {}

    // Keeps the solution if it is feasible, within the fleet cap and lexicographically better.
    void offer(const Solution& sol, bool feasible) {
        if (!feasible) return;
        const int veh = sol.numVehicles();
        if (maxVehicles_ > 0 && veh > maxVehicles_) return;
        const double dist = sol.distance(inst_);
        if (have_ && !Solution::lexBetter(veh, dist, best_.numVehicles(), bestDist_)) return;
        best_ = sol;
        bestDist_ = dist;
        have_ = true;
        result_.trace.push_back({timer_.elapsed(), veh, dist});
    }

    bool have() const { return have_; }
    const Solution& solution() const { return best_; }

private:
    const Instance& inst_;
    int maxVehicles_;
    const Timer& timer_;
    SolveResult& result_;
    Solution best_;
    double bestDist_ = 0;
    bool have_ = false;
};

// Order crossover: copies a random segment of `a`, then fills the other positions with
// the remaining customers in the order they appear in `b`, starting after the segment.
std::vector<int> orderCrossover(const std::vector<int>& a, const std::vector<int>& b, int numNodes, Rng& rng) {
    const int n = static_cast<int>(a.size());
    if (n < 2) return a;
    std::uniform_int_distribution<int> pick(0, n - 1);
    int i = pick(rng), j = pick(rng);
    if (i > j) std::swap(i, j);

    std::vector<int> child(n, -1);
    std::vector<char> inChild(numNodes, 0);
    for (int k = i; k <= j; ++k) {
        child[k] = a[k];
        inChild[a[k]] = 1;
    }
    int pos = (j + 1) % n;
    for (int k = 0; k < n; ++k) {
        const int c = b[(j + 1 + k) % n];
        if (inChild[c]) continue;
        while (child[pos] != -1) pos = (pos + 1) % n;
        child[pos] = c;
        inChild[c] = 1;
    }
    return child;
}

// One run of the memetic algorithm. Holds the state of the search; the public Solver
// interface is a thin wrapper around it.
class GaRun {
public:
    GaRun(const Instance& inst, const SolverConfig& cfg)
        : inst_(inst), cfg_(cfg), rng_(cfg.seed), ls_(inst), split_(inst),
          best_(inst, cfg.maxVehicles, timer_, result_), cap_(cfg.maxVehicles > 0 ? cfg.maxVehicles : -1) {
        for (int c = inst.firstCustomer(); c < inst.numNodes(); ++c) customers_.push_back(c);
    }

    SolveResult run() {
        seedFleetWithIls();
        initPopulation();
        while (timer_.elapsed() < cfg_.timeLimit) {
            ++generation_;
            fleetMinimisation();
            breed();
            pop_.reduceTo(kMu, penalties_.weights());
            if (cfg_.verbose && generation_ % 10 == 0) log();
        }
        result_.feasible = best_.have();
        result_.best = best_.have() ? best_.solution() : pop_.at(0).sol;
        result_.runtime = timer_.elapsed();
        return result_;
    }

private:
    // Crossover alone is poor at removing vehicles (children of 18-route parents
    // decode into 21+ routes), so a short ILS run fixes the fleet size first; the
    // GA then works at that fleet size (cap_) and keeps trying to lower it.
    void seedFleetWithIls() {
        SolverConfig ilsCfg = cfg_;
        ilsCfg.timeLimit = kIlsSeedShare * cfg_.timeLimit;
        ilsCfg.verbose = false;
        SolveResult ils = IteratedLocalSearch().solve(inst_, ilsCfg);
        if (!ils.feasible) return;
        best_.offer(ils.best, true);
        tightenCap(ils.best.numVehicles());
        pop_.add(makeIndividual(inst_, ils.best));
    }

    void tightenCap(int vehicles) {
        if (cap_ < 0 || vehicles < cap_) cap_ = vehicles;
    }

    // Split ignores the fleet size: dissolve the shortest routes into the others
    // (penalised insertion) until the solution fits the cap.
    void enforceCap(Solution& sol) {
        if (cap_ < 0) return;
        sol.removeEmptyRoutes();
        while (sol.numVehicles() > cap_ &&
               dissolveRoute(inst_, sol, shortestRouteIndex(inst_, sol), penalties_.weights(), rng_)) {
            sol.removeEmptyRoutes();
        }
    }

    void initPopulation() {
        for (int k = pop_.size(); k < kMu; ++k) {
            Solution sol;
            if (k < kInitialGreedy) {
                sol = buildInitialSolution(inst_, rng_);
                best_.offer(sol, sol.feasible(inst_));
            } else {
                std::vector<int> tour = customers_;
                std::shuffle(tour.begin(), tour.end(), rng_);
                sol = split_.decode(tour, penalties_.weights());
            }
            pop_.add(educate(std::move(sol)));
        }
    }

    // Local search with the current penalties, then wrapped as an individual.
    Individual educate(Solution sol) {
        enforceCap(sol);
        ls_.run(sol, penalties_.weights(), rng_, cap_);
        Individual ind = makeIndividual(inst_, std::move(sol));
        best_.offer(ind.sol, ind.feasible);
        return ind;
    }

    // Tries to remove one vehicle from the best feasible solution. Done every generation
    // during the first share of the budget, then every kFleetEvery generations.
    void fleetMinimisation() {
        const bool early = timer_.elapsed() < kFleetShare * cfg_.timeLimit;
        if (!best_.have() || (!early && generation_ % kFleetEvery != 0)) return;
        Solution trial = best_.solution();
        if (!ls_.tryEliminateRoute(trial, rng_, timer_, cfg_.timeLimit)) return;
        best_.offer(trial, true);
        tightenCap(trial.numVehicles());
        pop_.add(makeIndividual(inst_, std::move(trial)));
    }

    // Generates kLambda offspring from the current population: tournament selection,
    // OX crossover, split decoding and education. Parents come from the first `base` members.
    void breed() {
        const int base = pop_.size();
        const std::vector<double> fit = pop_.fitness(penalties_.weights(), base);
        for (int k = 0; k < kLambda && timer_.elapsed() < cfg_.timeLimit; ++k) {
            const int a = tournament(fit, base);
            const int b = tournament(fit, base);
            const std::vector<int> tour = orderCrossover(pop_.at(a).tour, pop_.at(b).tour, inst_.numNodes(), rng_);
            Individual child = educate(split_.decode(tour, penalties_.weights()));
            penalties_.record(child.feasible);
            pop_.add(std::move(child));
        }
    }

    // Binary tournament on biased fitness (lower wins).
    int tournament(const std::vector<double>& fit, int base) {
        std::uniform_int_distribution<int> pick(0, base - 1);
        const int x = pick(rng_);
        const int y = pick(rng_);
        return fit[x] <= fit[y] ? x : y;
    }

    void log() const {
        std::cerr << "[ga] gen=" << generation_ << " t=" << timer_.elapsed() << " pop=" << pop_.size();
        if (best_.have())
            std::cerr << " best=" << best_.solution().numVehicles() << "/" << best_.solution().distance(inst_);
        std::cerr << '\n';
    }

    const Instance& inst_;
    const SolverConfig& cfg_;
    Timer timer_;
    Rng rng_;
    SolveResult result_;
    LocalSearch ls_;
    Split split_;
    Population pop_;
    AdaptivePenalties penalties_;
    BestTracker best_;
    std::vector<int> customers_;
    int cap_;
    int generation_ = 0;
};

}  // namespace

Solution Split::decode(const std::vector<int>& tour, const PenaltyWeights& w) const {
    const int n = static_cast<int>(tour.size());
    std::vector<double> pot(n + 1, std::numeric_limits<double>::infinity());
    std::vector<int> pred(n + 1, 0);
    std::vector<Route> segment(n + 1);  // repaired route of the best segment ending at i
    pot[0] = 0;
    for (int i = 1; i <= n; ++i) {
        Route seg;  // tour[j .. i-1], grown towards the front
        for (int j = i - 1; j >= 0 && i - j <= kMaxRouteCustomers; --j) {
            seg.insert(seg.begin(), tour[j]);
            // Battery deficits are penalised, not repaired: repairing every candidate
            // with charging stations costs ~0.6 s per split on the 100-customer
            // instances. Local search inserts the stations on the decoded routes.
            const double cost = pot[j] + penalisedCost(evaluateRoute(inst_, seg), w);
            if (cost < pot[i]) {
                pot[i] = cost;
                pred[i] = j;
                segment[i] = seg;
            }
        }
    }
    Solution sol;
    for (int i = n; i > 0; i = pred[i]) sol.routes.push_back(std::move(segment[i]));
    return sol;
}

std::vector<int> Split::encode(const Solution& sol, const Instance& inst) {
    std::vector<int> tour;
    for (const Route& r : sol.routes)
        for (int j : r)
            if (inst.isCustomer(j)) tour.push_back(j);
    return tour;
}

SolveResult MemeticGA::solve(const Instance& inst, const SolverConfig& cfg) {
    GaRun run(inst, cfg);
    return run.run();
}

}  // namespace evrptw
