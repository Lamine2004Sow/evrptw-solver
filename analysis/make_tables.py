#!/usr/bin/env python3
"""Write Markdown result tables to results/tables/.

Outputs:
  per_instance.md  one row per (instance, method): best-of-seeds vs BKS, mean/std over seeds
  summary.md       per method and instance class: best-of-seeds and mean-over-seeds gaps

Gaps are lexicographic: delta_veh = vehicles - BKS vehicles; the distance gap is
only reported for runs/instances using the same number of vehicles as the BKS.

Example:
    python3 analysis/make_tables.py results/raw/main.csv --bks data/bks.csv
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (DEFAULT_BKS, TABLES_DIR, fmt, load_bks, load_results,  # noqa: E402
                    md_table, per_instance_stats, summary_stats, write_text)


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("results", nargs="+", help="result CSV file(s) written by run_experiments.py")
    p.add_argument("--bks", default=DEFAULT_BKS, help="BKS CSV (instance,vehicles,distance,...)")
    p.add_argument("--out-dir", default=TABLES_DIR, help="output directory for the Markdown files")
    p.add_argument("--prefix", default="", help="optional prefix for output file names")
    return p.parse_args(argv)


def build_per_instance(records):
    headers = ["Instance", "Method", "Runs (feas.)", "BKS veh", "BKS dist",
               "Best veh", "Best dist", "dVeh", "Gap % (same fleet)",
               "Mean veh", "Mean dist", "Std dist", "Mean runtime (s)"]
    rows = []
    for p in records:
        same = p["same_fleet_best"]
        gap = fmt(p["dist_gap_best"], 2, pct=True) if same else (
            "n/a (fleet differs)" if same is False else "-")
        rows.append([
            p["instance"], p["method"], f"{p['n_feasible']}/{p['n_runs']}",
            fmt(p["bks_vehicles"], 0), fmt(p["bks_distance"], 2),
            fmt(p["best_vehicles"], 0), fmt(p["best_distance"], 2),
            fmt(p["delta_veh_best"], 0, signed=True), gap,
            fmt(p["mean_vehicles"], 2), fmt(p["mean_distance"], 2),
            fmt(p["std_distance"], 2), fmt(p["mean_runtime"], 2),
        ])
    text = ("# Per-instance results\n\n"
            "Best-of-seeds is the lexicographic minimum (vehicles, distance) over feasible runs. "
            "`-` means no BKS value is available.\n\n")
    return text + md_table(headers, rows)


def build_summary(stats):
    headers = ["Method", "Class", "Instances", "With BKS",
               "Best: same fleet", "Best: mean dVeh", "Best: mean gap % (same fleet)",
               "Runs", "Runs: same fleet", "Runs: mean dVeh",
               "Runs: mean gap %", "Runs: std gap %", "Mean runtime (s)"]
    rows = []
    for s in stats:
        rows.append([
            s["method"], s["group"], s["n_instances"], s["n_with_bks"],
            fmt(s["best_same_fleet_pct"], 1, pct=True),
            fmt(s["best_mean_delta_veh"], 2, signed=True),
            fmt(s["best_mean_dist_gap"], 2, pct=True),
            s["n_runs"], fmt(s["run_same_fleet_pct"], 1, pct=True),
            fmt(s["run_mean_delta_veh"], 2, signed=True),
            fmt(s["run_mean_dist_gap"], 2, pct=True),
            fmt(s["run_std_dist_gap"], 2, pct=True),
            fmt(s["mean_runtime"], 2),
        ])
    text = ("# Summary by method and instance class\n\n"
            "Class ALL aggregates every instance of the method. Distance gaps use only runs "
            "(or instances) whose vehicle count equals the BKS count.\n\n")
    return text + md_table(headers, rows)


def main(argv=None):
    args = parse_args(argv)
    rows = load_results(args.results)
    if not rows:
        print("error: no result rows loaded", file=sys.stderr)
        return 1
    bks = load_bks(args.bks)
    if not any(v.get("vehicles") is not None for v in bks.values()):
        print("note: no BKS values available; gap columns will show '-'", file=sys.stderr)

    records = per_instance_stats(rows, bks)
    records.sort(key=lambda p: (p["instance"], p["method"]))
    stats = summary_stats(rows, bks)
    pre = args.prefix
    write_text(os.path.join(args.out_dir, f"{pre}per_instance.md"), build_per_instance(records))
    write_text(os.path.join(args.out_dir, f"{pre}summary.md"), build_summary(stats))
    return 0


if __name__ == "__main__":
    sys.exit(main())
