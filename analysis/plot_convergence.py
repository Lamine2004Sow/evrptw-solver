#!/usr/bin/env python3
"""Plot convergence traces (time, vehicles, distance) of one or more instances.

Trace files are looked up as <trace-dir>/<instance>_<method>_s<seed>.csv, the
naming used by run_experiments.py. For each instance one figure is written with:
  top:    vehicles over time (step plot), one line per run
  bottom: best distance over time, one line per run, BKS distance as a dashed line

Example:
    python3 analysis/plot_convergence.py --trace-dir results/traces \
        --instance c101C10 r201_21 --methods ils ga
"""

import argparse
import csv
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import DEFAULT_BKS, FIGURES_DIR, ensure_dir, load_bks, warn  # noqa: E402

COLORS = {"greedy": "#7f7f7f", "ils": "#1f77b4", "ga": "#d62728", "milp": "#2ca02c"}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--trace-dir", default=os.path.join(os.path.dirname(FIGURES_DIR), "traces"),
                   help="directory with trace CSVs")
    p.add_argument("--instance", nargs="+", required=True,
                   help="instance name(s), e.g. c101C10 r201_21")
    p.add_argument("--methods", nargs="+", default=None,
                   help="methods to include (default: all found in trace-dir)")
    p.add_argument("--bks", default=DEFAULT_BKS, help="BKS CSV for the reference line")
    p.add_argument("--out-dir", default=FIGURES_DIR, help="output directory")
    p.add_argument("--dpi", type=int, default=150)
    return p.parse_args(argv)


def read_trace(path):
    """Return (times, vehicles, distances) lists; tolerant to column naming."""
    times, vehs, dists = [], [], []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return times, vehs, dists
        names = [h.strip().lower() for h in header]
        has_header = any(not re.fullmatch(r"[-\d.eE+]+", h.strip()) for h in header)
        idx = {"time": 0, "vehicles": 1, "distance": 2}
        if has_header:
            for key in idx:
                if key in names:
                    idx[key] = names.index(key)
        else:
            reader = [header] + list(reader)
        for row in reader:
            try:
                times.append(float(row[idx["time"]]))
                vehs.append(float(row[idx["vehicles"]]))
                dists.append(float(row[idx["distance"]]))
            except (ValueError, IndexError):
                continue
    return times, vehs, dists


def find_runs(trace_dir, instance, methods):
    """Return {method: [(seed, path), ...]} for an instance."""
    pattern = os.path.join(trace_dir, f"{instance}_*_s*.csv")
    runs = {}
    for path in sorted(glob.glob(pattern)):
        m = re.fullmatch(re.escape(instance) + r"_(.+)_s(\d+)\.csv", os.path.basename(path))
        if not m:
            continue
        method, seed = m.group(1), int(m.group(2))
        if methods and method not in methods:
            continue
        runs.setdefault(method, []).append((seed, path))
    return runs


def main(argv=None):
    args = parse_args(argv)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("error: matplotlib is required for plotting", file=sys.stderr)
        return 1
    if not os.path.isdir(args.trace_dir):
        print(f"error: trace directory not found: {args.trace_dir}", file=sys.stderr)
        return 1
    bks = load_bks(args.bks)
    ensure_dir(args.out_dir)
    status = 0

    for inst in args.instance:
        runs = find_runs(args.trace_dir, inst, args.methods)
        if not runs:
            warn(f"no traces found for {inst} in {args.trace_dir}; skipped")
            status = 1
            continue
        fig, (ax_v, ax_d) = plt.subplots(2, 1, figsize=(8, 7), sharex=True,
                                         gridspec_kw={"height_ratios": [1, 2]})
        for method in sorted(runs):
            color = COLORS.get(method, None)
            for seed, path in sorted(runs[method]):
                times, vehs, dists = read_trace(path)
                if not times:
                    warn(f"empty trace skipped: {path}")
                    continue
                label = method if seed == sorted(runs[method])[0][0] else None
                ax_v.step(times, vehs, where="post", color=color, alpha=0.6, label=label)
                ax_d.step(times, dists, where="post", color=color, alpha=0.6, label=label)
        bk = bks.get(inst)
        if bk and bk.get("distance") is not None:
            ax_d.axhline(bk["distance"], ls="--", color="black", lw=1,
                         label=f"BKS ({bk['vehicles']} veh, {bk['distance']:.2f})")
        if bk and bk.get("vehicles") is not None:
            ax_v.axhline(bk["vehicles"], ls="--", color="black", lw=1, label="BKS vehicles")
        ax_v.set_ylabel("vehicles")
        ax_d.set_ylabel("distance")
        ax_d.set_xlabel("time (s)")
        ax_v.grid(alpha=0.3)
        ax_d.grid(alpha=0.3)
        ax_v.legend(fontsize=8)
        ax_d.legend(fontsize=8)
        ax_v.set_title(f"{inst}: convergence over {sum(len(v) for v in runs.values())} runs "
                       f"(all seeds)", fontsize=10)
        fig.tight_layout()
        out = os.path.join(args.out_dir, f"convergence_{inst}.png")
        fig.savefig(out, dpi=args.dpi)
        plt.close(fig)
        print(f"wrote {out}")
    return status


if __name__ == "__main__":
    sys.exit(main())
