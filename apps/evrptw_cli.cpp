// Command-line entry point.
//
//   evrptw solve <instance> [--method ils] [--seed 1] [--time 10] [--max-vehicles K]
//                           [--opt key=value ...] [--out sol.txt] [--trace trace.csv] [--verbose]
//   evrptw check <instance> <solution>
//   evrptw info  <instance>
//
// `solve` prints one CSV line on stdout (see kCsvHeader); the result is always
// passed through the independent checker first.
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "evrptw/Checker.hpp"
#include "evrptw/Instance.hpp"
#include "evrptw/Solver.hpp"

using namespace evrptw;

namespace {

const char* kCsvHeader =
    "instance,method,seed,time_limit,vehicles,distance,feasible,runtime,lower_bound,optimal";

int usage() {
    std::cerr << "usage:\n"
                 "  evrptw solve <instance> [--method M] [--seed S] [--time T] [--max-vehicles K]\n"
                 "               [--opt key=value ...]  (milp: copies=N, warmstart=SECONDS)\n"
                 "               [--out FILE] [--trace FILE] [--header] [--verbose]\n"
                 "  evrptw check <instance> <solution>\n"
                 "  evrptw info  <instance>\n"
                 "methods:";
    for (const auto& m : availableSolvers()) std::cerr << ' ' << m;
    std::cerr << '\n';
    return 2;
}

// Parses "--flag value" pairs; repeated "--opt key=value" go to `solverOptions`.
std::map<std::string, std::string> parseOptions(int argc, char** argv, int from,
                                                std::map<std::string, std::string>& solverOptions) {
    std::map<std::string, std::string> opt;
    for (int i = from; i < argc; ++i) {
        std::string a = argv[i];
        if (a.rfind("--", 0) != 0) throw std::invalid_argument("unexpected argument: " + a);
        if (a == "--opt" && i + 1 < argc) {
            std::string kv = argv[++i];
            auto eq = kv.find('=');
            if (eq == std::string::npos) throw std::invalid_argument("--opt expects key=value: " + kv);
            solverOptions[kv.substr(0, eq)] = kv.substr(eq + 1);
        } else if (a == "--verbose" || a == "--header") opt[a] = "1";
        else if (i + 1 < argc) opt[a] = argv[++i];
        else throw std::invalid_argument("missing value for " + a);
    }
    return opt;
}

int cmdSolve(int argc, char** argv) {
    if (argc < 3) return usage();
    Instance inst = Instance::load(argv[2]);
    SolverConfig cfg;
    auto opt = parseOptions(argc, argv, 3, cfg.options);

    std::string method = opt.count("--method") ? opt["--method"] : "ils";
    if (opt.count("--seed")) cfg.seed = std::stoull(opt["--seed"]);
    if (opt.count("--time")) cfg.timeLimit = std::stod(opt["--time"]);
    if (opt.count("--max-vehicles")) cfg.maxVehicles = std::stoi(opt["--max-vehicles"]);
    cfg.verbose = opt.count("--verbose") > 0;

    auto solver = makeSolver(method);
    SolveResult res = solver->solve(inst, cfg);

    CheckReport rep = checkSolution(inst, res.best);
    if (res.feasible && !rep.ok) {
        std::cerr << "ERROR: solver claims feasibility but the checker disagrees:\n";
        for (const auto& e : rep.errors) std::cerr << "  " << e << '\n';
        return 1;
    }
    if (opt.count("--out")) res.best.write(inst, opt["--out"]);
    if (opt.count("--trace")) {
        std::ofstream t(opt["--trace"]);
        t.precision(10);
        t << "time,vehicles,distance\n";
        for (const auto& p : res.trace) t << p.time << ',' << p.vehicles << ',' << p.distance << '\n';
    }

    if (opt.count("--header")) std::cout << kCsvHeader << '\n';
    std::cout.precision(10);
    std::cout << inst.name() << ',' << method << ',' << cfg.seed << ',' << cfg.timeLimit << ','
              << (rep.ok ? rep.vehicles : -1) << ',' << (rep.ok ? rep.distance : -1.0) << ','
              << (rep.ok ? 1 : 0) << ',' << res.runtime << ',' << res.lowerBound << ','
              << (res.optimal ? 1 : 0) << '\n';
    return 0;
}

int cmdCheck(int argc, char** argv) {
    if (argc < 4) return usage();
    Instance inst = Instance::load(argv[2]);
    Solution sol = Solution::read(inst, argv[3]);
    CheckReport rep = checkSolution(inst, sol);
    std::cout.precision(10);
    std::cout << (rep.ok ? "FEASIBLE" : "INFEASIBLE") << " vehicles=" << rep.vehicles
              << " distance=" << rep.distance << '\n';
    for (const auto& e : rep.errors) std::cout << "  " << e << '\n';
    return rep.ok ? 0 : 1;
}

int cmdInfo(int argc, char** argv) {
    if (argc < 3) return usage();
    Instance inst = Instance::load(argv[2]);
    std::cout << inst.name() << ": " << inst.numCustomers() << " customers, " << inst.numStations()
              << " stations, Q=" << inst.batteryCapacity() << " C=" << inst.loadCapacity()
              << " r=" << inst.consumptionRate() << " g=" << inst.rechargeRate()
              << " v=" << inst.velocity() << " horizon=" << inst.horizon() << '\n';
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    std::string cmd = argv[1];
    try {
        if (cmd == "solve") return cmdSolve(argc, argv);
        if (cmd == "check") return cmdCheck(argc, argv);
        if (cmd == "info") return cmdInfo(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return usage();
}
