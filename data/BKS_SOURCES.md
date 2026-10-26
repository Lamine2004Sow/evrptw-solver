# Best-known solutions (BKS) for the Schneider E-VRPTW instances

File: `data/bks.csv` (columns: instance, vehicles, distance, source, url). 92 rows; empty vehicles/distance means no value was read.

## Source used

**Schneider, M., Stenger, A., Goeke, D. (2014). The Electric Vehicle-Routing Problem with Time Windows and Recharging Stations. Transportation Science 48(4):500-520. DOI 10.1287/trsc.2013.0490.**
Read from the copy at http://web4.ensiie.fr/~faye/mpro/MPRO_reseau/Projet_2020/The%20electric%20vehicle%20routing%20problem%20with%20time%20windows%20and%20recharging%20stations.pdf (a third-party host of the paper; the DOI above is the canonical reference).

Model checks made on the text: the objective is hierarchical (minimise vehicles first, then total distance, section 3); recharging is a complete recharge (recharge times "for a complete recharge", section 3). This is the full-recharge E-VRPTW, so partial-recharge results (e.g. Keskin & Çatay 2016) were not used.

### Small instances (36, 5/10/15 customers): Table 3
- Columns used: CPLEX `m`, `Lbest(S)` and VNS/TS `m`, `Lbest(S)` (best of 10 runs).
- Per instance, the value is the lexicographically best of the two (fewest vehicles, then lowest distance). Differences found: c206C5 (VNS/TS 242.55 < CPLEX 242.56), c202C15 (383.61 < 383.62), rc204C15 (VNS/TS 1 vehicle, 384.86 < CPLEX 407.45), rc108C5 (CPLEX 1 vehicle, 253.93 < VNS/TS 2 vehicles).
- Caveat: CPLEX values are optimal or best upper bounds within the 7200 s limit. The paper states that for most 15-customer instances CPLEX did not prove optimality, so these small-instance values are best-known rather than proven optimal.

### Large instances (56, 100 customers): Table 4
- Columns used: the `BKS` block, `m` and `Lbest(S)` (the best solution found across all of the paper's runs, per the caption "gaps are calculated to the best known solution").
- Mapped to the repo file names by adding the `_21` suffix (e.g. `c101` -> `c101_21`). The paper text does not spell out the station count, so the `_21` mapping is by file-name convention.

## Not used
- Keskin & Çatay (2016) partial-recharge results: excluded by the full-recharge requirement.
- A third-party GitHub repository (0neblaze/FURP-2026-Yiyang-GUO-EVRP-TW) that refers to a `schneider_best_known.csv`: its file could not be retrieved (HTTP 404), and it cites a partial-recharge source for the 100-customer set, so nothing was taken from it.

## Gaps and open items
- No later paper with an improved full-recharge BKS was verified. Desaulniers et al. (2016), Operations Research 64(6), was located, but its computational tables were not readable from the sources available, so no values were taken from it.
- The Schneider et al. 2014 BKS for large instances are heuristic values, not proven optima.

## Correction: rc108C5

Table 3 reports m = 1 for CPLEX and m = 2 for VNS/TS, both at distance 253.93. A single
vehicle at that distance would have to chain the two routes through the depot station S0,
and both orders violate time windows (arrival 388 and 382 at the depot, due 240; check with
`evrptw check`). Our MILP also proves 2 vehicles optimal with 2 station copies. We therefore
treat m = 1 as a typo and use m = 2.
