#!/usr/bin/env python3
"""Independent re-verification of solution files, sharing no code with the C++ solver.

Re-simulates every route of every solution (full linear recharge, waiting allowed,
no lateness) and checks capacity, battery, time windows and that each customer is
served exactly once. Exit code 1 if any solution is infeasible.

Example:
    python3 analysis/verify_solutions.py results/best_solutions --data-dir data/instances
"""
import argparse
import math
import os
import sys

EPS = 1e-6


def load_instance(path):
    nodes, params = {}, {}
    with open(path) as f:
        lines = f.read().splitlines()[1:]
    for line in lines:
        if not line.strip():
            continue
        if "/" in line:
            params[line[0]] = float(line.split("/")[1])
            continue
        t = line.split()
        nodes[t[0]] = dict(type=t[1], x=float(t[2]), y=float(t[3]), demand=float(t[4]),
                           ready=float(t[5]), due=float(t[6]), service=float(t[7]))
    return nodes, params


def verify(nodes, p, routes):
    """Returns (errors, vehicles, distance)."""
    Q, C, r, g, v = p["Q"], p["C"], p["r"], p["g"], p["v"]
    depot = next(k for k, n in nodes.items() if n["type"] == "d")
    errors, served, total = [], [], 0.0
    for k, route in enumerate(routes):
        if route[0] != depot or route[-1] != depot:
            errors.append(f"route {k} does not start and end at the depot")
        t, battery, load = nodes[depot]["ready"], Q, 0.0
        for a, b in zip(route, route[1:]):
            A, B = nodes[a], nodes[b]
            d = math.dist((A["x"], A["y"]), (B["x"], B["y"]))
            total += d
            t += d / v
            battery -= r * d
            if battery < -EPS:
                errors.append(f"route {k}: battery {battery:.4f} at {b}")
            t = max(t, B["ready"])
            if t > B["due"] + EPS:
                errors.append(f"route {k}: arrival {t:.4f} at {b} after due {B['due']}")
            if B["type"] == "c":
                served.append(b)
                load += B["demand"]
                t += B["service"]
            elif B["type"] == "f":
                t += g * (Q - battery)
                battery = Q
        if load > C + EPS:
            errors.append(f"route {k}: load {load} > capacity {C}")
    customers = sorted(k for k, n in nodes.items() if n["type"] == "c")
    if sorted(served) != customers:
        errors.append("customers not served exactly once")
    return errors, len(routes), total


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("solutions", help="directory of <instance>.txt solution files")
    ap.add_argument("--data-dir", default="data/instances")
    args = ap.parse_args()

    bad = 0
    files = sorted(f for f in os.listdir(args.solutions) if f.endswith(".txt"))
    for f in files:
        name = f[:-4]
        nodes, params = load_instance(os.path.join(args.data_dir, name + ".txt"))
        with open(os.path.join(args.solutions, f)) as fh:
            routes = [l.split() for l in fh if l.strip() and not l.startswith("#")]
        errors, veh, dist = verify(nodes, params, routes)
        status = "OK " if not errors else "BAD"
        print(f"{status} {name:12s} vehicles={veh:3d} distance={dist:.2f}")
        for e in errors[:5]:
            print("     ", e)
        bad += bool(errors)
    print(f"{len(files) - bad}/{len(files)} solutions feasible")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
