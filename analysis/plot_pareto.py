#!/usr/bin/env python3
"""Plot the vehicles-distance Pareto front of each instance (step plot).

Input CSV needs the columns instance,vehicles,distance. An optional 'method'
column gives one front per method. Every point (solution) is drawn faintly and
the non-dominated points (fewer-or-equal vehicles AND lower-or-equal distance,
at least one strict) form the step front.

One PNG is written per instance to results/figures/pareto_<instance>.png.

Example:
    python3 analysis/plot_pareto.py results/pareto/c101C10.csv --instances c101C10
"""

import argparse
import csv
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import FIGURES_DIR, ensure_dir, warn  # noqa: E402

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b", "#17becf"]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("csv", help="CSV with columns instance,vehicles,distance [,method]")
    p.add_argument("--instances", nargs="+", default=None,
                   help="restrict to these instance names (default: all)")
    p.add_argument("--out-dir", default=FIGURES_DIR, help="output directory")
    p.add_argument("--dpi", type=int, default=150)
    return p.parse_args(argv)


def pareto_front(points):
    """points: list of (vehicles, distance). Return the non-dominated subset sorted by vehicles."""
    front = []
    for v, d in sorted(set(points)):
        if not front or d < front[-1][1]:
            front.append((v, d))
    return front


def load_points(path, wanted):
    data = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            inst = (row.get("instance") or "").strip()
            if not inst or (wanted and inst not in wanted):
                continue
            try:
                veh = int(float(row["vehicles"]))
                dist = float(row["distance"])
            except (KeyError, TypeError, ValueError):
                continue
            method = (row.get("method") or "front").strip() or "front"
            data.setdefault(inst, {}).setdefault(method, []).append((veh, dist))
    return data


def main(argv=None):
    args = parse_args(argv)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.ticker import MaxNLocator
    except ImportError:
        print("error: matplotlib is required for plotting", file=sys.stderr)
        return 1
    if not os.path.isfile(args.csv):
        print(f"error: file not found: {args.csv}", file=sys.stderr)
        return 1
    data = load_points(args.csv, set(args.instances) if args.instances else None)
    if not data:
        print("error: no usable rows (need instance,vehicles,distance)", file=sys.stderr)
        return 1
    ensure_dir(args.out_dir)

    for inst in sorted(data):
        fig, ax = plt.subplots(figsize=(7, 5))
        for k, method in enumerate(sorted(data[inst])):
            pts = data[inst][method]
            color = COLORS[k % len(COLORS)]
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=12, color=color, alpha=0.25)
            front = pareto_front(pts)
            if not front:
                continue
            xs = [p[0] for p in front]
            ys = [p[1] for p in front]
            ax.step(xs, ys, where="post", color=color, lw=1.8, label=method)
            ax.scatter(xs, ys, s=30, color=color, zorder=3)
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
        ax.set_xlabel("vehicles (primary objective)")
        ax.set_ylabel("total distance")
        ax.set_title(f"{inst}: vehicles-distance Pareto front", fontsize=10)
        ax.grid(alpha=0.3)
        if len(data[inst]) > 1 or "front" not in data[inst]:
            ax.legend(fontsize=8)
        fig.tight_layout()
        out = os.path.join(args.out_dir, f"pareto_{inst}.png")
        fig.savefig(out, dpi=args.dpi)
        plt.close(fig)
        print(f"wrote {out}")
    missing = set(args.instances or []) - set(data)
    if missing:
        warn(f"no rows for instances: {sorted(missing)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
