# displib-solver

Solver for the DISPLIB train dispatching benchmark.

## Status
Resource-aware greedy scheduler. 9 of 112 instances verified
feasible against the official checker. Known bug under
investigation.

## Setup
Instances, the official verifier and competition solutions are
not included. Download from https://displib.github.io and place
them in `problems/`, `displib_verify.py`, and `solutions/`.

## Files
- `look.py` — solver and batch runner
- `check.py` — runs the official verifier across all instances
- `milp.py` — MILP implementation from DISPLIB paper