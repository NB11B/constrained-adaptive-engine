# Benchmarking

CAE validation is based on repeatable seeded simulator trials. The goal is not just to produce a visually plausible flight path, but to collect simple pass/fail and safety metrics that can be compared across commits.

## Smoke Test

Build the native shared library, then run:

```bash
python3 constrained_adaptive_engine_bridge.py
```

Expected result:

```text
[SUCCESS] Memory bounds match perfectly.
```

## Real Environment Integration Test

```bash
python3 test_real_env.py
```

This runs the current real-environment integration harness against the configured Swarm environment.

## Focused Subset Benchmark

The active optimization branch is focused on the terrain set most relevant to the current safety and landing behavior:

```bash
python3 benchmark_subset.py --terrains 3 4 5 6 --trials 1 --fixed
```

Terrain IDs:

```text
3 = Mountain
4 = Village
5 = Warehouse
6 = Forest
```

## Full Benchmark

```bash
python3 benchmark_cae.py --trials 1 --fixed
```

Increase `--trials` only after the fast deterministic gate is passing.

## Acceptance Gates

For the current feature branch, the focused subset should be interpreted with these gates:

- Mountain: no collision; minimum distance improves relative to the safe-timeout baseline.
- Village: success, no collision.
- Warehouse: success preferred; no-collision timeout acceptable only while tuning terminal commit.
- Forest: success, no collision.
- Total collisions: zero.

## Metrics to Record

Each benchmark run should preserve:

- branch name
- commit SHA
- benchmark command
- terrain IDs
- trial count
- seed policy
- success count
- timeout count
- collision count
- per-terrain minimum distance
- notable phase labels or guards triggered

## Interpreting Local Swarm Results

Some local Swarm/Crazyflow builds expose a 4-wide action space and do not consume yaw commands. Those runs are still useful for collision and landing-envelope testing, but they are not identical to validator-style 5-wide action-space runs.

When reporting results, specify whether the run used:

```text
4-wide local action space: [dir_x, dir_y, dir_z, speed]
5-wide validator-style action space: [dir_x, dir_y, dir_z, speed, yaw]
```

## Recommended Result Format

```text
Branch: feature/mountain-optimization
Commit: <sha>
Command: python3 benchmark_subset.py --terrains 3 4 5 6 --trials 1 --fixed
Swarm action space: (1, 4) or (1, 5)

Mountain: <SUCCESS|TIMEOUT|COLLISION>, steps=<n>, min_dist=<m>
Village:  <SUCCESS|TIMEOUT|COLLISION>, steps=<n>, min_dist=<m>
Warehouse:<SUCCESS|TIMEOUT|COLLISION>, steps=<n>, min_dist=<m>
Forest:   <SUCCESS|TIMEOUT|COLLISION>, steps=<n>, min_dist=<m>

Total collisions: <n>
Notes: <phase/guard observations>
```
