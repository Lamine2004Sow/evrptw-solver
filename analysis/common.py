"""Shared helpers for the E-VRPTW analysis scripts (stdlib + numpy/matplotlib only).

Objective is lexicographic: fewer vehicles first, then lower total distance.
A run is therefore only comparable on distance with the BKS when it uses the
same number of vehicles (`same_fleet`).
"""

import csv
import math
import os
import re
import statistics
import sys
from collections import defaultdict

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
RESULTS_DIR = os.path.join(REPO_ROOT, "results")
TABLES_DIR = os.path.join(RESULTS_DIR, "tables")
FIGURES_DIR = os.path.join(RESULTS_DIR, "figures")
DEFAULT_BKS = os.path.join(REPO_ROOT, "data", "bks.csv")
DEFAULT_INSTANCES = os.path.join(REPO_ROOT, "data", "instances")

RESULT_FIELDS = [
    "instance", "method", "seed", "time_limit", "vehicles", "distance",
    "feasible", "runtime", "lower_bound", "optimal",
]
LARGE_GROUPS = ["C1", "C2", "R1", "R2", "RC1", "RC2"]
SMALL_GROUPS = ["C5", "C10", "C15"]
ALL_GROUPS = SMALL_GROUPS + LARGE_GROUPS


def warn(msg):
    print(f"[warning] {msg}", file=sys.stderr)


def ensure_dir(path):
    os.makedirs(path, exist_ok=True)
    return path


def _to_float(text):
    if text is None:
        return None
    text = str(text).strip()
    if text == "" or text.lower() in ("nan", "none", "null", "-"):
        return None
    try:
        value = float(text)
    except ValueError:
        return None
    return None if math.isnan(value) else value


def _to_bool(text):
    return str(text).strip().lower() in ("1", "true", "yes", "y", "t")


def load_results(paths):
    """Load one or more result CSVs (columns as written by run_experiments.py).

    Returns a list of dicts with typed values. Missing numeric values are None.
    Rows of several files are concatenated; exact duplicates (same instance,
    method, seed) keep the last occurrence.
    """
    if isinstance(paths, str):
        paths = [paths]
    rows = {}
    for path in paths:
        if not os.path.isfile(path):
            warn(f"results file not found: {path}")
            continue
        with open(path, newline="") as f:
            for raw in csv.DictReader(f):
                try:
                    seed = int(float(raw["seed"]))
                except (KeyError, TypeError, ValueError):
                    warn(f"skipping malformed row in {path}: {raw}")
                    continue
                row = {
                    "instance": raw["instance"].strip(),
                    "method": raw["method"].strip(),
                    "seed": seed,
                    "time_limit": _to_float(raw.get("time_limit")),
                    "vehicles": None if _to_float(raw.get("vehicles")) is None
                    else int(_to_float(raw.get("vehicles"))),
                    "distance": _to_float(raw.get("distance")),
                    "feasible": _to_bool(raw.get("feasible", "1")),
                    "runtime": _to_float(raw.get("runtime")),
                    "lower_bound": _to_float(raw.get("lower_bound")),
                    "optimal": _to_bool(raw.get("optimal", "0")),
                }
                rows[(row["instance"], row["method"], seed)] = row
    return list(rows.values())


def load_bks(path=DEFAULT_BKS):
    """Return {instance: {"vehicles": int|None, "distance": float|None, "source": str}}."""
    bks = {}
    if not path or not os.path.isfile(path):
        warn(f"BKS file not found: {path}; gaps will be skipped")
        return bks
    with open(path, newline="") as f:
        for raw in csv.DictReader(f):
            veh = _to_float(raw.get("vehicles"))
            dist = _to_float(raw.get("distance"))
            bks[raw["instance"].strip()] = {
                "vehicles": None if veh is None else int(veh),
                "distance": dist,
                "source": (raw.get("source") or "").strip(),
            }
    return bks


def instance_group(name):
    """Map an instance name to its class label.

    Large (100 customers, '_21' suffix): C1, C2, R1, R2, RC1, RC2.
    Small (5/10/15 customers, e.g. 'c101C5'): C5, C10, C15.
    Returns None for names that match neither pattern.
    """
    m = re.fullmatch(r"(rc|c|r)([12])\d{2}_21", name, flags=re.IGNORECASE)
    if m:
        return m.group(1).upper() + m.group(2)
    m = re.fullmatch(r"(?:rc|c|r)\d{3}C(\d+)", name, flags=re.IGNORECASE)
    if m:
        return "C" + m.group(1)
    return None


def instance_size_class(name):
    return "large" if name.endswith("_21") else "small"


def _sorted_key(group):
    return ALL_GROUPS.index(group) if group in ALL_GROUPS else len(ALL_GROUPS)


def gap_fields(vehicles, distance, bk):
    """Lexicographic gap of one solution w.r.t. one BKS entry.

    delta_veh     = vehicles - bks_vehicles (negative means fewer vehicles than BKS)
    dist_gap_pct  = 100 * (distance - bks_distance) / bks_distance
    same_fleet    = True when the vehicle count equals the BKS count
    Distance gaps are only meaningful when same_fleet is True; callers should
    filter on it. Missing BKS values give None everywhere.
    """
    out = {"delta_veh": None, "dist_gap_pct": None, "same_fleet": None}
    if bk is None or bk.get("vehicles") is None or vehicles is None:
        return out
    out["delta_veh"] = vehicles - bk["vehicles"]
    out["same_fleet"] = out["delta_veh"] == 0
    if bk.get("distance") and distance is not None:
        out["dist_gap_pct"] = 100.0 * (distance - bk["distance"]) / bk["distance"]
    return out


def run_gaps(rows, bks):
    """Attach gap fields to each feasible run. Runs without BKS get None gaps."""
    enriched = []
    for r in rows:
        if not r["feasible"]:
            continue
        g = gap_fields(r["vehicles"], r["distance"], bks.get(r["instance"]))
        enriched.append({**r, **g, "group": instance_group(r["instance"])})
    return enriched


def _mean(values):
    values = [v for v in values if v is not None]
    return statistics.fmean(values) if values else None


def _std(values):
    values = [v for v in values if v is not None]
    return statistics.stdev(values) if len(values) > 1 else (0.0 if values else None)


def per_instance_stats(rows, bks):
    """One record per (instance, method).

    Best-of-seeds is the lexicographic minimum (vehicles, distance) over feasible
    runs. Mean-over-seeds uses all feasible runs.
    """
    buckets = defaultdict(list)
    for r in rows:
        buckets[(r["instance"], r["method"])].append(r)
    records = []
    for (inst, method), runs in sorted(buckets.items()):
        feas = [r for r in runs if r["feasible"] and r["vehicles"] is not None
                and r["distance"] is not None]
        bk = bks.get(inst)
        rec = {
            "instance": inst, "method": method, "group": instance_group(inst),
            "n_runs": len(runs), "n_feasible": len(feas),
            "bks_vehicles": bk.get("vehicles") if bk else None,
            "bks_distance": bk.get("distance") if bk else None,
            "best_vehicles": None, "best_distance": None,
            "mean_vehicles": None, "mean_distance": None, "std_distance": None,
            "mean_runtime": _mean([r["runtime"] for r in runs]),
            "delta_veh_best": None, "dist_gap_best": None, "same_fleet_best": None,
            "mean_delta_veh": None, "mean_dist_gap_same_fleet": None,
            "n_same_fleet": 0,
        }
        if feas:
            best = min(feas, key=lambda r: (r["vehicles"], r["distance"]))
            rec["best_vehicles"] = best["vehicles"]
            rec["best_distance"] = best["distance"]
            rec["mean_vehicles"] = _mean([r["vehicles"] for r in feas])
            rec["mean_distance"] = _mean([r["distance"] for r in feas])
            rec["std_distance"] = _std([r["distance"] for r in feas])
            g = gap_fields(best["vehicles"], best["distance"], bk)
            rec["delta_veh_best"] = g["delta_veh"]
            rec["dist_gap_best"] = g["dist_gap_pct"]
            rec["same_fleet_best"] = g["same_fleet"]
            per_run = [gap_fields(r["vehicles"], r["distance"], bk) for r in feas]
            rec["mean_delta_veh"] = _mean([x["delta_veh"] for x in per_run])
            same = [x["dist_gap_pct"] for x in per_run if x["same_fleet"]]
            rec["n_same_fleet"] = len(same)
            rec["mean_dist_gap_same_fleet"] = _mean(same)
        records.append(rec)
    return records


def summary_stats(rows, bks):
    """Aggregate per (method, class) and per method (class 'ALL').

    Best-of-seeds block: computed over instances (best run per instance).
    Mean-over-seeds block: computed over all feasible runs.
    """
    per_inst = per_instance_stats(rows, bks)
    runs = run_gaps(rows, bks)
    keys = sorted({(p["method"], p["group"] or "?") for p in per_inst})
    out = []
    for method in sorted({m for m, _ in keys}):
        for group in sorted({g for m, g in keys if m == method}, key=_sorted_key) + ["ALL"]:
            if group == "ALL":
                ps = [p for p in per_inst if p["method"] == method]
                rs = [r for r in runs if r["method"] == method]
            else:
                ps = [p for p in per_inst if p["method"] == method and (p["group"] or "?") == group]
                rs = [r for r in runs if r["method"] == method and (r["group"] or "?") == group]
            with_bks = [p for p in ps if p["bks_vehicles"] is not None]
            best_same = [p for p in with_bks if p["same_fleet_best"]]
            run_with_bks = [r for r in rs if r["delta_veh"] is not None]
            run_same = [r for r in run_with_bks if r["same_fleet"]]
            out.append({
                "method": method,
                "group": group,
                "n_instances": len(ps),
                "n_with_bks": len(with_bks),
                "best_same_fleet_pct": (100.0 * len(best_same) / len(with_bks)) if with_bks else None,
                "best_mean_delta_veh": _mean([p["delta_veh_best"] for p in with_bks]),
                "best_mean_dist_gap": _mean([p["dist_gap_best"] for p in best_same]),
                "n_runs": len(rs),
                "run_same_fleet_pct": (100.0 * len(run_same) / len(run_with_bks)) if run_with_bks else None,
                "run_mean_delta_veh": _mean([r["delta_veh"] for r in run_with_bks]),
                "run_mean_dist_gap": _mean([r["dist_gap_pct"] for r in run_same]),
                "run_std_dist_gap": _std([r["dist_gap_pct"] for r in run_same]),
                "mean_runtime": _mean([r["runtime"] for r in rs]),
            })
    return out


def fmt(value, digits=2, pct=False, signed=False):
    if value is None:
        return "-"
    if isinstance(value, float) and math.isnan(value):
        return "-"
    spec = f"{'+' if signed else ''}.{digits}f"
    text = format(value, spec)
    return text + ("%" if pct else "")


def md_table(headers, rows):
    lines = ["| " + " | ".join(headers) + " |",
             "|" + "|".join("---" for _ in headers) + "|"]
    for row in rows:
        lines.append("| " + " | ".join(str(c) for c in row) + " |")
    return "\n".join(lines) + "\n"


def write_text(path, text):
    ensure_dir(os.path.dirname(path))
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"wrote {path}")
    return path


def load_instance(path):
    """Parse a Schneider E-VRPTW instance file.

    Returns {"nodes": {id: {"type": 'd'|'f'|'c', "x", "y", "demand", "ready",
    "due", "service"}}, "params": {...}}. Node types: d depot, f station, c customer.
    """
    nodes, params = {}, {}
    with open(path, encoding="utf-8") as f:
        for line in f:
            parts = line.split()
            if len(parts) >= 8 and parts[1] in ("d", "f", "c") and parts[0] != "StringID":
                try:
                    nodes[parts[0]] = {
                        "type": parts[1], "x": float(parts[2]), "y": float(parts[3]),
                        "demand": float(parts[4]), "ready": float(parts[5]),
                        "due": float(parts[6]), "service": float(parts[7]),
                    }
                except ValueError:
                    continue
            else:
                m = re.match(r"^\s*([A-Za-z])\s+.*?/([-\d.]+)/\s*$", line)
                if m:
                    params[m.group(1)] = float(m.group(2))
    return {"nodes": nodes, "params": params}


def load_solution(path):
    """Read a solution file: '#' lines are comments, every other non-empty line is a route."""
    routes = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            nodes = s.split()
            if len(nodes) >= 2:
                routes.append(nodes)
    return routes
