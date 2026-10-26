#!/usr/bin/env bash
# Full experimental campaign (resumable: rerun to finish interrupted runs).
# Heuristics: 10 seeds; 30 s on the 56 large instances, 5 s on the 36 small ones.
# MILP: small instances, 900 s, 3 station copies, 5 s ILS warm start.
set -euo pipefail
cd "$(dirname "$0")/.."
JOBS=${JOBS:-4}
RUN="python3 scripts/run_experiments.py --save-solutions results/solutions --trace-dir results/traces"

$RUN --methods ils ga --instances large --seeds 10 --time 30 --jobs "$JOBS" --out results/raw/large.csv
$RUN --methods ils ga --instances small --seeds 10 --time 5 --jobs "$JOBS" --out results/raw/small.csv
$RUN --methods milp --instances small --seeds 1 --time 900 --jobs 3 --opt warmstart=5 --out results/raw/milp_small.csv
$RUN --methods milp --instances small --seeds 1 --time 600 --jobs 5 --opt warmstart=5 --opt copies=2 --out results/raw/milp_small_2copies.csv --save-solutions results/solutions_2copies
