// Bi-objective front (vehicles, distance) by epsilon-constraint on the fleet size.
//
//   evrptw_pareto <instance> [--method ils|ga] [--seed S] [--time-per-point T]
//                 [--extra K] [--out front.csv]
//
// Method: solve unconstrained to get the minimum fleet m*, then solve with the fleet
// capped at K = m*, ..., m*+extra. Every solution is re-checked, and only the
// non-dominated (vehicles, distance) points are kept.
//
// The front is small by nature: on these instances the fleet size only takes a few
// values around m*, so there are at most extra+1 candidate fleet sizes and the
// non-dominated set has at most that many points.
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "evrptw/Checker.hpp"
#include "evrptw/Instance.hpp"
#include "evrptw/Solver.hpp"

using namespace evrptw;

namespace {

struct Point {
    int vehicles;
    double distance;
    Solution sol;
};

int usage() {
    std::cerr << "usage: evrptw_pareto <instance> [--method ils|ga] [--seed S] [--time-per-point T]\n"
                 "                     [--extra K] [--out front.csv]\n";
    return 2;
}

// Keeps the points not dominated in (vehicles, distance), both minimised.
std::vector<Point> nonDominated(std::vector<Point> pts) {
    std::sort(pts.begin(), pts.end(), [](const Point& a, const Point& b) {
        if (a.vehicles != b.vehicles) return a.vehicles < b.vehicles;
        return a.distance < b.distance;
    });
    std::vector<Point> front;
    double bestDist = std::numeric_limits<double>::infinity();
    for (const Point& p : pts) {
        if (p.distance < bestDist - 1e-7) {
            front.push_back(p);
            bestDist = p.distance;
        }
    }
    return front;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string path = argv[1];
    std::map<std::string, std::string> opt;
    try {
        for (int i = 2; i < argc; ++i) {
            const std::string a = argv[i];
            if (a.rfind("--", 0) != 0 || i + 1 >= argc) return usage();
            opt[a] = argv[++i];
        }
        const std::string method = opt.count("--method") ? opt["--method"] : "ils";
        if (method != "ils" && method != "ga") throw std::invalid_argument("method must be ils or ga");
        const std::uint64_t seed = opt.count("--seed") ? std::stoull(opt["--seed"]) : 1;
        const double timePerPoint = opt.count("--time-per-point") ? std::stod(opt["--time-per-point"]) : 10.0;
        const int extra = opt.count("--extra") ? std::stoi(opt["--extra"]) : 3;

        Instance inst = Instance::load(path);
        auto solver = makeSolver(method);
        SolverConfig cfg;
        cfg.seed = seed;
        cfg.timeLimit = timePerPoint;

        std::vector<Point> candidates;
        auto addIfValid = [&](const SolveResult& res, const char* label) {
            const CheckReport rep = checkSolution(inst, res.best);
            if (!rep.ok) {
                std::cerr << label << ": infeasible, skipped\n";
                return;
            }
            candidates.push_back({rep.vehicles, rep.distance, res.best});
            std::cerr << label << ": vehicles=" << rep.vehicles << " distance=" << rep.distance << '\n';
        };

        const SolveResult base = solver->solve(inst, cfg);
        const int mStar = base.best.numVehicles();
        addIfValid(base, "unconstrained");

        for (int k = mStar; k <= mStar + extra; ++k) {
            cfg.maxVehicles = k;
            const SolveResult res = solver->solve(inst, cfg);
            addIfValid(res, ("K=" + std::to_string(k)).c_str());
        }

        const std::vector<Point> front = nonDominated(candidates);
        std::ostream* out = &std::cout;
        std::ofstream file;
        if (opt.count("--out")) {
            file.open(opt["--out"]);
            if (!file) throw std::runtime_error("cannot write " + opt["--out"]);
            out = &file;
        }
        *out << "instance,vehicles,distance,method,seed\n";
        out->precision(10);
        for (const Point& p : front)
            *out << inst.name() << ',' << p.vehicles << ',' << p.distance << ',' << method << ',' << seed << '\n';
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
