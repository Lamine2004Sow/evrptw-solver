#include "evrptw/milp/MilpSolver.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "evrptw/meta/ILS.hpp"
#include "gurobi_c++.h"

namespace evrptw {

namespace {

enum class Kind { Start, Customer, Station, End };

// Drops a station visit that immediately repeats the previous one (a no-op).
Route withoutRepeatedStations(const Instance& inst, const Route& route) {
    Route out;
    for (int v : route)
        if (!(inst.isStation(v) && !out.empty() && out.back() == v)) out.push_back(v);
    return out;
}

}  // namespace

MilpSolver::MilpSolver(int copiesPerStation, bool hierarchical)
    : copiesPerStation_(copiesPerStation), hierarchical_(hierarchical) {
    if (copiesPerStation_ < 1) throw std::invalid_argument("MilpSolver: copiesPerStation must be >= 1");
}

SolveResult MilpSolver::solve(const Instance& inst, const SolverConfig& cfg) {
    Timer timer;
    SolveResult res;
    const int copies = static_cast<int>(cfg.option("copies", copiesPerStation_));
    if (copies < 1) throw std::invalid_argument("MilpSolver: copies must be >= 1");

    // Optional warm start: a short ILS run provides the incumbent (MIP start).
    Solution warm;
    const double warmTime = cfg.option("warmstart", 0.0);
    if (warmTime > 0) {
        SolverConfig ilsCfg = cfg;
        ilsCfg.timeLimit = std::min(warmTime, cfg.timeLimit);
        ilsCfg.verbose = false;
        SolveResult ils = IteratedLocalSearch().solve(inst, ilsCfg);
        if (ils.feasible) warm = ils.best;
    }

    try {
        // ------------------------------------------------------------ model nodes
        // 0: start depot; 1..nC: customers; then station copies; last: end depot.
        std::vector<int> instOf;   // model node -> instance node index
        std::vector<Kind> kind;
        instOf.push_back(inst.depot());
        kind.push_back(Kind::Start);
        for (int c = inst.firstCustomer(); c < inst.numNodes(); ++c) {
            instOf.push_back(c);
            kind.push_back(Kind::Customer);
        }
        for (int s = inst.firstStation(); s < inst.firstCustomer(); ++s)
            for (int k = 0; k < copies; ++k) {
                instOf.push_back(s);
                kind.push_back(Kind::Station);
            }
        instOf.push_back(inst.depot());
        kind.push_back(Kind::End);

        const int N = static_cast<int>(instOf.size());
        const int START = 0;
        const int END = N - 1;

        const double Q = inst.batteryCapacity();
        const double C = inst.loadCapacity();
        const double r = inst.consumptionRate();
        const double g = inst.rechargeRate();
        const double L0 = inst.horizon();

        auto ready = [&](int i) { return inst.node(instOf[i]).ready; };
        auto due = [&](int i) { return inst.node(instOf[i]).due; };
        auto service = [&](int i) { return kind[i] == Kind::Customer ? inst.node(instOf[i]).service : 0.0; };
        auto demand = [&](int i) { return kind[i] == Kind::Customer ? inst.node(instOf[i]).demand : 0.0; };
        auto travel = [&](int i, int j) { return inst.travelTime(instOf[i], instOf[j]); };

        // ------------------------------------------------------------ arc set (with pruning)
        std::vector<char> arcOK(N * N, 0);
        auto idx = [N](int i, int j) { return i * N + j; };
        for (int i = 0; i < N; ++i) {
            if (kind[i] == Kind::End) continue;
            for (int j = 0; j < N; ++j) {
                if (j == i || kind[j] == Kind::Start) continue;
                if (i == START && j == END) continue;
                if (instOf[i] == instOf[j] && kind[i] == Kind::Station && kind[j] == Kind::Station) continue;
                if (ready(i) + service(i) + travel(i, j) > due(j) + 1e-9) continue;        // time windows
                if (kind[i] != Kind::Customer && r * inst.dist(instOf[i], instOf[j]) > Q) continue;  // battery
                if (demand(i) + demand(j) > C) continue;                                       // capacity
                arcOK[idx(i, j)] = 1;
            }
        }

        // ------------------------------------------------------------ Gurobi environment
        GRBEnv env(true);
        env.set(GRB_StringParam_LogFile, "");
        env.set(GRB_IntParam_OutputFlag, cfg.verbose ? 1 : 0);
        env.set(GRB_IntParam_Seed, static_cast<int>(cfg.seed % 2000000000ULL));
        env.set(GRB_DoubleParam_TimeLimit, std::max(1.0, cfg.timeLimit - timer.elapsed()));
        env.set(GRB_IntParam_Threads, 1);
        env.start();

        GRBModel model(env);

        // ------------------------------------------------------------ variables
        std::vector<GRBVar> x(N * N);
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < N; ++j)
                if (arcOK[idx(i, j)]) x[idx(i, j)] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);

        std::vector<GRBVar> tau(N), y(N), u(N);
        for (int i = 0; i < N; ++i) {
            tau[i] = model.addVar(ready(i), due(i), 0.0, GRB_CONTINUOUS);     // service start time
            y[i] = (kind[i] == Kind::Start) ? model.addVar(Q, Q, 0.0, GRB_CONTINUOUS)  // battery on arrival
                                            : model.addVar(0.0, Q, 0.0, GRB_CONTINUOUS);
            u[i] = model.addVar(0.0, C, 0.0, GRB_CONTINUOUS);                 // remaining load on arrival
        }
        model.update();

        // ------------------------------------------------------------ degree / flow
        GRBLinExpr vehicles, distance;
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (!arcOK[idx(i, j)]) continue;
                distance += inst.dist(instOf[i], instOf[j]) * x[idx(i, j)];
                if (i == START) vehicles += x[idx(i, j)];
            }
        }

        for (int i = 1; i < END; ++i) {
            GRBLinExpr out, in;
            for (int j = 0; j < N; ++j) {
                if (arcOK[idx(i, j)]) out += x[idx(i, j)];
                if (arcOK[idx(j, i)]) in += x[idx(j, i)];
            }
            if (kind[i] == Kind::Customer) {
                model.addConstr(out == 1);
            } else {
                model.addConstr(out <= 1);
            }
            model.addConstr(in == out);
        }

        // ------------------------------------------------------------ time
        for (int i = 0; i < N; ++i) {
            if (kind[i] == Kind::End) continue;
            for (int j = 0; j < N; ++j) {
                if (!arcOK[idx(i, j)]) continue;
                const GRBVar& xv = x[idx(i, j)];
                if (kind[i] == Kind::Station) {
                    // tau_i + t x + g (Q - y_i) - (L0 + gQ)(1 - x) <= tau_j
                    const double M = L0 + g * Q;
                    model.addConstr(tau[i] + (travel(i, j) + M) * xv - g * y[i] - tau[j] <= L0);
                } else {
                    // tau_i + (s_i + t) x - M (1 - x) <= tau_j
                    const double M = std::max(0.0, due(i) + service(i) + travel(i, j) - ready(j));
                    model.addConstr(tau[i] + (service(i) + travel(i, j) + M) * xv - tau[j] <= M);
                }
            }
        }

        // ------------------------------------------------------------ battery
        for (int i = 0; i < END; ++i) {
            for (int j = 1; j < N; ++j) {
                if (!arcOK[idx(i, j)]) continue;
                const GRBVar& xv = x[idx(i, j)];
                const double re = r * inst.dist(instOf[i], instOf[j]);
                if (kind[i] == Kind::Customer) {
                    // y_j <= y_i - r d x + Q (1 - x)
                    model.addConstr(y[j] - y[i] + (re + Q) * xv <= Q);
                } else {
                    // y_j <= Q - r d x   (start depot or station: full battery on departure)
                    model.addConstr(y[j] + re * xv <= Q);
                }
            }
        }

        // ------------------------------------------------------------ load
        for (int i = 0; i < END; ++i) {
            for (int j = 1; j < N; ++j) {
                if (!arcOK[idx(i, j)]) continue;
                const GRBVar& xv = x[idx(i, j)];
                // u_j <= u_i - q_i x + C (1 - x)
                model.addConstr(u[j] - u[i] + (demand(i) + C) * xv <= C);
            }
        }

        if (cfg.maxVehicles > 0) model.addConstr(vehicles <= cfg.maxVehicles);

        // ------------------------------------------------------------ MIP start
        // Maps each station visit of the warm solution to the next unused copy; the
        // start is skipped if a station is visited more often than it has copies.
        if (!warm.routes.empty()) {
            std::vector<int> modelOfCustomer(inst.numNodes(), -1);
            std::vector<std::vector<int>> freeCopies(inst.numNodes());
            for (int i = N - 2; i > 0; --i) {
                if (kind[i] == Kind::Customer) modelOfCustomer[instOf[i]] = i;
                else freeCopies[instOf[i]].push_back(i);
            }
            std::vector<double> start(N * N, 0.0);
            bool ok = true;
            for (const Route& r : warm.routes) {
                int prev = START;
                Route path = withoutRepeatedStations(inst, r);
                path.push_back(-1);  // end depot
                for (int v : path) {
                    int m;
                    if (v < 0) m = END;
                    else if (inst.isCustomer(v)) m = modelOfCustomer[v];
                    else if (!freeCopies[v].empty()) { m = freeCopies[v].back(); freeCopies[v].pop_back(); }
                    else { ok = false; break; }
                    if (!arcOK[idx(prev, m)]) { ok = false; break; }
                    start[idx(prev, m)] = 1.0;
                    prev = m;
                }
                if (!ok) break;
            }
            if (ok)
                for (int i = 0; i < N; ++i)
                    for (int j = 0; j < N; ++j)
                        if (arcOK[idx(i, j)]) x[idx(i, j)].set(GRB_DoubleAttr_Start, start[idx(i, j)]);
        }

        // ------------------------------------------------------------ objective
        model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
        if (hierarchical_) {
            model.setObjectiveN(vehicles, 0, 2);
            model.setObjectiveN(distance, 1, 1);
        } else {
            model.setObjective(distance, GRB_MINIMIZE);
        }

        model.optimize();

        // ------------------------------------------------------------ results
        const int status = model.get(GRB_IntAttr_Status);
        const int solCount = model.get(GRB_IntAttr_SolCount);

        Solution sol;
        if (solCount > 0) {
            auto isSet = [&](int i, int j) {
                return arcOK[idx(i, j)] && x[idx(i, j)].get(GRB_DoubleAttr_X) > 0.5;
            };
            for (int j = 1; j < N; ++j) {
                if (!isSet(START, j)) continue;
                Route route;
                int cur = j;
                while (kind[cur] != Kind::End) {
                    if (static_cast<int>(route.size()) > N) throw std::runtime_error("MILP: cycle in route extraction");
                    route.push_back(instOf[cur]);
                    int next = -1;
                    for (int k = 0; k < N && next < 0; ++k)
                        if (isSet(cur, k)) next = k;
                    if (next < 0) throw std::runtime_error("MILP: broken route in solution");
                    cur = next;
                }
                sol.routes.push_back(route);
            }
        }

        res.best = sol;
        res.feasible = solCount > 0;
        res.optimal = status == GRB_OPTIMAL;
        res.lowerBound = -1;
        if (res.optimal || (!hierarchical_ && status == GRB_TIME_LIMIT)) {
            try {
                res.lowerBound = model.get(GRB_DoubleAttr_ObjBound);
            } catch (const GRBException&) {
                res.lowerBound = -1;  // ObjBound is not queryable for some multi-objective models
            }
        }
        // At OPTIMAL the bound equals the objective value of the final stage (distance).
        if (res.optimal && res.lowerBound < 0 && res.feasible) res.lowerBound = sol.distance(inst);

        res.runtime = timer.elapsed();
        if (res.feasible) {
            res.trace.push_back({res.runtime, sol.numVehicles(), sol.distance(inst)});
        }
    } catch (const GRBException& e) {
        throw std::runtime_error("Gurobi error " + std::to_string(e.getErrorCode()) + ": " + e.getMessage());
    }

    return res;
}

}  // namespace evrptw
