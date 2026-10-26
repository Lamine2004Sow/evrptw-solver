#!/usr/bin/env python3
"""Gap plots per method and instance class.

  gap_boxplot.png    distance gap (%) to the BKS, one box per method and class.
                     Only runs using the BKS number of vehicles are included,
                     because the distance of a solution with a different fleet
                     is not comparable under the lexicographic objective.
  delta_vehicles.png mean vehicle difference (run - BKS) per method and class,
                     with the min/max over seeds as error bars.

Instances without a BKS value are skipped with a note.

Example:
    python3 analysis/plot_gaps.py results/raw/main.csv --bks data/bks.csv
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import (ALL_GROUPS, DEFAULT_BKS, FIGURES_DIR, ensure_dir, load_bks,  # noqa: E402
                    load_results, run_gaps)

COLORS = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e", "#8c564b", "#17becf"]


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("results", nargs="+", help="result CSV file(s)")
    p.add_argument("--bks", default=DEFAULT_BKS, help="BKS CSV")
    p.add_argument("--out-dir", default=FIGURES_DIR, help="output directory")
    p.add_argument("--dpi", type=int, default=150)
    return p.parse_args(argv)


def _layout(methods, groups):
    """Return {(method, group): x_position} with one cluster per group."""
    width = 0.8 / max(1, len(methods))
    pos = {}
    for gi, g in enumerate(groups):
        for mi, m in enumerate(methods):
            pos[(m, g)] = gi + (mi - (len(methods) - 1) / 2) * width
    return pos, width


def main(argv=None):
    args = parse_args(argv)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Patch
    except ImportError:
        print("error: matplotlib is required for plotting", file=sys.stderr)
        return 1
    rows = load_results(args.results)
    bks = load_bks(args.bks)
    runs = [r for r in run_gaps(rows, bks) if r["delta_veh"] is not None and r["group"]]
    if not runs:
        print("note: no feasible runs with a BKS value and a class label; nothing to plot",
              file=sys.stderr)
        return 0

    methods = sorted({r["method"] for r in runs})
    groups = [g for g in ALL_GROUPS if any(r["group"] == g for r in runs)]
    colors = {m: COLORS[i % len(COLORS)] for i, m in enumerate(methods)}
    pos, width = _layout(methods, groups)
    ensure_dir(args.out_dir)

    # --- distance gap boxplot (same fleet only)
    same = [r for r in runs if r["same_fleet"] and r["dist_gap_pct"] is not None]
    if same:
        fig, ax = plt.subplots(figsize=(max(7, 1.2 * len(groups) + 2), 5))
        for m in methods:
            for g in groups:
                vals = [r["dist_gap_pct"] for r in same if r["method"] == m and r["group"] == g]
                if not vals:
                    continue
                bp = ax.boxplot([vals], positions=[pos[(m, g)]], widths=width * 0.9,
                                patch_artist=True, showfliers=True,
                                flierprops={"markersize": 3})
                for patch in bp["boxes"]:
                    patch.set_facecolor(colors[m])
                    patch.set_alpha(0.6)
        ax.set_xticks(range(len(groups)))
        ax.set_xticklabels(groups)
        ax.axhline(0, color="black", lw=0.8)
        ax.set_ylabel("distance gap to BKS (%)")
        ax.set_title("Distance gap per run (same fleet as BKS only)", fontsize=10)
        ax.legend(handles=[Patch(facecolor=colors[m], alpha=0.6, label=m) for m in methods],
                  fontsize=8)
        ax.grid(alpha=0.3, axis="y")
        fig.tight_layout()
        out = os.path.join(args.out_dir, "gap_boxplot.png")
        fig.savefig(out, dpi=args.dpi)
        plt.close(fig)
        print(f"wrote {out}")
    else:
        print("note: no same-fleet runs with a distance gap; gap_boxplot.png skipped",
              file=sys.stderr)

    # --- mean delta vehicles bars with min/max error bars
    fig, ax = plt.subplots(figsize=(max(7, 1.2 * len(groups) + 2), 5))
    for m in methods:
        for g in groups:
            vals = [r["delta_veh"] for r in runs if r["method"] == m and r["group"] == g]
            if not vals:
                continue
            mean = sum(vals) / len(vals)
            lo, hi = mean - min(vals), max(vals) - mean
            ax.bar(pos[(m, g)], mean, width=width * 0.9, color=colors[m], alpha=0.7,
                   yerr=[[lo], [hi]], capsize=3)
    ax.set_xticks(range(len(groups)))
    ax.set_xticklabels(groups)
    ax.axhline(0, color="black", lw=0.8)
    ax.set_ylabel("vehicles - BKS vehicles (mean over runs)")
    ax.set_title("Vehicle difference to BKS (negative = fewer vehicles than BKS)", fontsize=10)
    ax.legend(handles=[Patch(facecolor=colors[m], alpha=0.7, label=m) for m in methods], fontsize=8)
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    out = os.path.join(args.out_dir, "delta_vehicles.png")
    fig.savefig(out, dpi=args.dpi)
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
