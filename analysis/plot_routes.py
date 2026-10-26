#!/usr/bin/env python3
"""Draw the routes of one solution on the instance map.

Depot: black square. Recharging stations: green triangles. Customers: dots.
Each route has its own colour; the route is drawn as a polyline through
depot, customers and stations in visiting order.

Solution format: lines starting with '#' are comments; every other non-empty
line is one route of node ids (e.g. "D0 C12 S3 C30 D0").

Example:
    python3 analysis/plot_routes.py --instance data/instances/c101C5.txt \
        --solution results/solutions/c101C5_ils_s1.txt --title "c101C5 ILS seed 1"
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import FIGURES_DIR, ensure_dir, load_instance, load_solution, warn  # noqa: E402


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--instance", required=True, help="instance file (data/instances/*.txt)")
    p.add_argument("--solution", required=True, help="solution file (route per line)")
    p.add_argument("--out", default=None,
                   help="output PNG (default: results/figures/routes_<solution name>.png)")
    p.add_argument("--title", default=None, help="figure title")
    p.add_argument("--dpi", type=int, default=150)
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.lines import Line2D
    except ImportError:
        print("error: matplotlib is required for plotting", file=sys.stderr)
        return 1

    inst = load_instance(args.instance)
    nodes = inst["nodes"]
    if not nodes:
        print(f"error: no nodes parsed from {args.instance}", file=sys.stderr)
        return 1
    routes = load_solution(args.solution)
    if not routes:
        warn(f"no routes found in {args.solution}; drawing the instance only")

    fig, ax = plt.subplots(figsize=(8, 8))
    cust = [(n, v) for n, v in nodes.items() if v["type"] == "c"]
    stat = [(n, v) for n, v in nodes.items() if v["type"] == "f"]
    if cust:
        ax.scatter([v["x"] for _, v in cust], [v["y"] for _, v in cust],
                   s=14, c="#444444", zorder=3)
    if stat:
        ax.scatter([v["x"] for _, v in stat], [v["y"] for _, v in stat],
                   s=90, marker="^", c="#2ca02c", edgecolors="black", linewidths=0.5, zorder=4)
    depots = [v for v in nodes.values() if v["type"] == "d"]
    for d in depots:
        ax.scatter([d["x"]], [d["y"]], s=160, marker="s", c="black", zorder=5)

    cmap = plt.get_cmap("tab20")
    unknown = set()
    for k, route in enumerate(routes):
        xs, ys = [], []
        for node in route:
            if node in nodes:
                xs.append(nodes[node]["x"])
                ys.append(nodes[node]["y"])
            else:
                unknown.add(node)
        if len(xs) >= 2:
            ax.plot(xs, ys, "-", color=cmap(k % 20), linewidth=1.4, alpha=0.9, zorder=2)
    if unknown:
        warn(f"unknown node ids skipped: {sorted(unknown)[:10]}")

    handles = [
        Line2D([0], [0], marker="s", color="w", markerfacecolor="black", markersize=10, label="depot"),
        Line2D([0], [0], marker="^", color="w", markerfacecolor="#2ca02c", markeredgecolor="black",
               markersize=10, label="recharging station"),
        Line2D([0], [0], marker="o", color="w", markerfacecolor="#444444", markersize=6, label="customer"),
    ]
    ax.legend(handles=handles, loc="upper right", fontsize=8)
    n_stat_visits = sum(1 for r in routes for n in r if n.startswith("S"))
    title = args.title or os.path.splitext(os.path.basename(args.instance))[0]
    ax.set_title(f"{title}\n{len(routes)} routes, {n_stat_visits} station visits", fontsize=10)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(alpha=0.3)
    fig.tight_layout()

    out = args.out
    if out is None:
        stem = os.path.splitext(os.path.basename(args.solution))[0]
        out = os.path.join(ensure_dir(FIGURES_DIR), f"routes_{stem}.png")
    ensure_dir(os.path.dirname(os.path.abspath(out)))
    fig.savefig(out, dpi=args.dpi)
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
