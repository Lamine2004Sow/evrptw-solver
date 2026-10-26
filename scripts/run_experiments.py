#!/usr/bin/env python3
"""Run (instance x method x seed) experiments of the E-VRPTW solver in parallel.

Each run is a subprocess call to the C++ CLI:
    <binary> solve <instance> --method M --seed S --time T --out SOL --trace TRACE

Result rows are appended to a single CSV (header written once). Runs whose
(instance, method, seed) key is already present in the output CSV are skipped,
so an interrupted campaign can simply be restarted with the same arguments.

Example:
    python3 scripts/run_experiments.py --methods ils ga --instances all \
        --seeds 10 --time 30 --jobs 4 --out results/raw/main.csv \
        --save-solutions results/solutions/ --trace-dir results/traces/
"""

import argparse
import concurrent.futures as cf
import csv
import fnmatch
import os
import re
import subprocess
import sys
import threading
import time

RESULT_HEADER = [
    "instance", "method", "seed", "time_limit", "vehicles", "distance",
    "feasible", "runtime", "lower_bound", "optimal",
]
METHODS = ["greedy", "ils", "ga", "milp"]
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        description="Run E-VRPTW solver experiments in parallel and collect results in a CSV.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--methods", nargs="+", default=["ils"], choices=METHODS,
                   help="solver methods to run")
    p.add_argument("--instances", nargs="+", default=["all"],
                   help="'small' (no _21 suffix), 'large' (_21 suffix), 'all', "
                        "or glob patterns on instance names (e.g. 'r1*', 'c101C*'). "
                        "Several values are combined (union).")
    p.add_argument("--seeds", default="10",
                   help="'N' for seeds 1..N, 'A..B' for a range, or a comma list like '1,4,9'")
    p.add_argument("--time", type=float, default=30.0, help="time limit per run in seconds")
    p.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2),
                   help="number of runs executed in parallel")
    p.add_argument("--binary", default=os.path.join(REPO_ROOT, "build", "evrptw"),
                   help="path to the evrptw CLI binary")
    p.add_argument("--data-dir", default=os.path.join(REPO_ROOT, "data", "instances"),
                   help="directory containing the instance .txt files")
    p.add_argument("--out", default=os.path.join(REPO_ROOT, "results", "raw", "run.csv"),
                   help="output CSV (appended to, header written once)")
    p.add_argument("--save-solutions", default=None, metavar="DIR",
                   help="directory where solution files are saved (one per run)")
    p.add_argument("--trace-dir", default=None, metavar="DIR",
                   help="directory where convergence trace CSVs are saved (one per run)")
    p.add_argument("--timeout-margin", type=float, default=60.0,
                   help="extra seconds allowed beyond --time before a run is killed")
    p.add_argument("--opt", action="append", default=[], metavar="KEY=VALUE",
                   help="solver option passed through to the binary (repeatable), e.g. copies=3")
    p.add_argument("--dry-run", action="store_true",
                   help="print the list of runs that would be executed and exit")
    return p.parse_args(argv)


def parse_seeds(text):
    text = text.strip()
    if re.fullmatch(r"\d+", text):
        n = int(text)
        if n < 1:
            raise ValueError("--seeds N must be >= 1")
        return list(range(1, n + 1))
    m = re.fullmatch(r"(\d+)\.\.(\d+)", text)
    if m:
        a, b = int(m.group(1)), int(m.group(2))
        if b < a:
            raise ValueError("--seeds range must be increasing")
        return list(range(a, b + 1))
    try:
        return [int(x) for x in text.split(",") if x.strip()]
    except ValueError as exc:
        raise ValueError(f"cannot parse --seeds '{text}'") from exc


def list_instances(data_dir):
    if not os.path.isdir(data_dir):
        raise FileNotFoundError(f"instance directory not found: {data_dir}")
    return sorted(f[:-4] for f in os.listdir(data_dir) if f.endswith(".txt"))


def select_instances(all_names, selectors):
    chosen = set()
    for sel in selectors:
        if sel == "all":
            chosen.update(all_names)
        elif sel == "small":
            chosen.update(n for n in all_names if not n.endswith("_21"))
        elif sel == "large":
            chosen.update(n for n in all_names if n.endswith("_21"))
        else:
            matched = [n for n in all_names if fnmatch.fnmatchcase(n, sel)]
            if not matched:
                print(f"warning: pattern '{sel}' matched no instance", file=sys.stderr)
            chosen.update(matched)
    return sorted(chosen)


def run_key(instance, method, seed):
    return (instance, method, str(seed))


def load_done_keys(path):
    done = set()
    if not os.path.isfile(path):
        return done
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            try:
                done.add(run_key(row["instance"], row["method"], int(row["seed"])))
            except (KeyError, TypeError, ValueError):
                continue
    return done


def parse_result_line(stdout):
    """Return the last CSV result line printed by the binary as a dict, or None."""
    result = None
    for line in stdout.splitlines():
        parts = [p.strip() for p in line.strip().split(",")]
        if len(parts) == len(RESULT_HEADER) and parts[0] != "instance":
            result = parts
    if result is None:
        return None
    return dict(zip(RESULT_HEADER, result))


def run_one(job, args):
    """Execute one run; returns (job, row_dict_or_None, error_message_or_None)."""
    instance, method, seed = job["instance"], job["method"], job["seed"]
    cmd = [args.binary, "solve", job["path"], "--method", method,
           "--seed", str(seed), "--time", str(args.time)]
    for kv in args.opt:
        cmd += ["--opt", kv]
    if job["solution"]:
        cmd += ["--out", job["solution"]]
    if job["trace"]:
        cmd += ["--trace", job["trace"]]
    t0 = time.monotonic()
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True,
                              timeout=args.time + args.timeout_margin)
    except subprocess.TimeoutExpired:
        return job, None, f"timeout after {args.time + args.timeout_margin:.0f}s"
    except OSError as exc:
        return job, None, f"cannot execute binary: {exc}"
    wall = time.monotonic() - t0
    if proc.returncode != 0:
        tail = (proc.stderr or proc.stdout).strip().splitlines()[-3:]
        return job, None, f"exit code {proc.returncode} ({wall:.1f}s): {' | '.join(tail)}"
    row = parse_result_line(proc.stdout)
    if row is None:
        return job, None, "no result line in stdout"
    # Canonical identifiers so that the resume key always matches the job list.
    row["instance"] = instance
    row["method"] = method
    row["seed"] = str(seed)
    return job, row, None


def main(argv=None):
    args = parse_args(argv)
    try:
        seeds = parse_seeds(args.seeds)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2
    if args.jobs < 1:
        print("error: --jobs must be >= 1", file=sys.stderr)
        return 2
    if not args.dry_run and not os.path.isfile(args.binary):
        print(f"error: binary not found: {args.binary} (build the project first)", file=sys.stderr)
        return 2

    instances = select_instances(list_instances(args.data_dir), args.instances)
    if not instances:
        print("error: no instance selected", file=sys.stderr)
        return 2

    out_dir = os.path.dirname(os.path.abspath(args.out))
    os.makedirs(out_dir, exist_ok=True)
    if args.save_solutions:
        os.makedirs(args.save_solutions, exist_ok=True)
    if args.trace_dir:
        os.makedirs(args.trace_dir, exist_ok=True)
    err_path = os.path.splitext(args.out)[0] + ".errors.log"

    all_jobs = []
    for inst in instances:
        for method in args.methods:
            for seed in seeds:
                stem = f"{inst}_{method}_s{seed}"
                all_jobs.append({
                    "instance": inst, "method": method, "seed": seed,
                    "path": os.path.join(args.data_dir, inst + ".txt"),
                    "solution": os.path.join(args.save_solutions, stem + ".txt")
                    if args.save_solutions else None,
                    "trace": os.path.join(args.trace_dir, stem + ".csv")
                    if args.trace_dir else None,
                })

    done = load_done_keys(args.out)
    todo = [j for j in all_jobs
            if run_key(j["instance"], j["method"], j["seed"]) not in done]
    print(f"runs planned: {len(all_jobs)}  already done: {len(all_jobs) - len(todo)}  "
          f"to run: {len(todo)}  jobs: {args.jobs}  time: {args.time}s", flush=True)

    if args.dry_run:
        for j in todo:
            print(f"{j['instance']} {j['method']} {j['seed']}")
        return 0
    if not todo:
        print("nothing to do")
        return 0

    write_header = not (os.path.isfile(args.out) and os.path.getsize(args.out) > 0)
    lock = threading.Lock()
    n_ok = n_err = 0
    with open(args.out, "a", newline="") as fout, \
            open(err_path, "a") as ferr, \
            cf.ThreadPoolExecutor(max_workers=args.jobs) as pool:
        writer = csv.writer(fout)
        if write_header:
            writer.writerow(RESULT_HEADER)
            fout.flush()
        futures = [pool.submit(run_one, j, args) for j in todo]
        for k, fut in enumerate(cf.as_completed(futures), start=1):
            job, row, error = fut.result()
            tag = f"{job['instance']} {job['method']} seed={job['seed']}"
            with lock:
                if row is not None:
                    writer.writerow([row[c] for c in RESULT_HEADER])
                    fout.flush()
                    n_ok += 1
                    msg = (f"veh={row['vehicles']} dist={row['distance']} "
                           f"feasible={row['feasible']} t={row['runtime']}s")
                else:
                    ferr.write(f"{tag}: {error}\n")
                    ferr.flush()
                    n_err += 1
                    msg = f"FAILED: {error}"
                print(f"[{k}/{len(todo)}] {tag}  {msg}", flush=True)
    print(f"finished: {n_ok} ok, {n_err} failed"
          + (f" (see {err_path})" if n_err else ""), flush=True)
    return 0 if n_err == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
