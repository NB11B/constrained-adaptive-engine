# Final Report: Adaptive Engine for Drone Swarms (Subnet 124)

## Introduction
This report details the recent updates to the adaptive engine for drone swarms, specifically targeting the Swarm Subnet 124 challenge. The primary goal of these modifications is to enhance the drone's autonomous flight capabilities, particularly in AGL-based landing on moving pads with minimal sensor overhead. This report covers the implemented changes, the validation process using `test_real_env.py`, and the benchmarking results from `benchmark_cae.py`.

## Implemented Changes
The following changes were implemented across various files in the `constrained-adaptive-engine` repository:

### `drone_agent.py`
*   Fixed Subnet label to **124**.
*   Changed depth max range from `30.0` to `20.0`.
*   Replaced brittle `state[137]` / `state[138:141]` with `state[-4]` / `state[-3:]`.
*   Fixed yaw output so it no longer divides by π twice.
*   Fixed calibration sign so `true_target = noisy_center + calibrated_offset`.
*   Added full mission-state reset so AGL/search state does not leak between benchmark seeds.
*   Added `close()` passthrough for bridge cleanup.

### `test_real_env.py`
*   Removed hard-coded local Swarm paths.
*   Added portable `SWARM_REPO=/path/to/swarm` support.
*   Switched AGL indexing to `state[-4]`.
*   Fixed yaw normalization.
*   Added `engine.close()` cleanup in `finally`.

### `constrained_adaptive_engine_bridge.py`
*   Bound native `adapt_free()`.
*   Added `close()`, context-manager support, and safe destructor cleanup.
*   Added controller-lifetime guard so calls after cleanup fail cleanly instead of using a released pointer.

### `src/psmsl_depth_processor.c`
*   Added Swarm depth minimum constant `0.50f`.
*   Corrected C-side depth reconstruction from normalized depth: `z_cam = 0.5 + d_norm * (max_range - 0.5)`.
*   This aligns native deprojection with Swarm’s normalized 0.5m–20m depth tensor.

### `benchmark_cae.py`
*   Removed legacy `ProcScene`, `evaluate_sotapilot`, and `evaluate_constrained_engine` imports.
*   Replaced benchmark path with the real Swarm harness through `run_real_env_trial()`.
*   CSV now records terrain, seed, status, steps, wall time, and min distance.

## Validation Results (`test_real_env.py`)
The `test_real_env.py` script was executed to validate the basic functionality of the updated adaptive engine. The log indicated a `TIMEOUT` at step 2999, with a minimum distance of 3.374m. This suggests that while the drone was able to navigate, it did not successfully land within the given time limit for the specific seed tested.

```
TERMINATED at step=2999 dist=3.47 term=False trunc=True
=> TIMEOUT | steps=3000 | min_dist=3.374m | time=26.37s
```

## Benchmark Results (`benchmark_cae.py`)
The `benchmark_cae.py` script was run with `--trials 1 --fixed` to evaluate the performance across different terrains using a fixed seed (1337). The results are summarized below:

| Terrain     | Terrain ID | Seed | Status    | Steps | Time (s) | Min Dist (m) |
|-------------|------------|------|-----------|-------|----------|--------------|
| City Map    | 1          | 1337 | TIMEOUT   | 3000  | 10.79    | 3.123        |
| Open/Valley | 2          | 1337 | TIMEOUT   | 3000  | 21.23    | 0.517        |
| Mountain    | 3          | 1337 | COLLISION | 1689  | 179.59   | 5.104        |
| Village     | 4          | 1337 | TIMEOUT   | 3000  | 83.20    | 73.840       |
| Warehouse   | 5          | 1337 | TIMEOUT   | 3000  | 176.18   | 3.261        |
| Forest      | 6          | 1337 | TIMEOUT   | 3000  | 179.52   | 0.520        |

**Summary:**
*   **Total Trials:** 6
*   **Successes:** 0 (0.0%)
*   **Collisions:** 1
*   **Timeouts:** 5

The benchmark results indicate that the current implementation is not consistently achieving successful landings across all terrains. Most trials resulted in timeouts, and one resulted in a collision. The minimum distance achieved in some terrains (e.g., Open/Valley, Forest) is relatively low, suggesting the drone is getting close to the target but not completing the landing sequence.

## Conclusion and Next Steps
The implemented changes address several critical aspects of the adaptive engine, including AGL calibration, target tracking, and mission state management. However, the validation and benchmark results highlight that further refinement is needed to ensure reliable landing performance across diverse terrains. The high rate of timeouts suggests that the landing trigger or descent logic might require further tuning, or the search pattern needs to be more efficient in locating the pad.

**Next Steps:**
1.  **Analyze Timeout Scenarios:** Investigate the specific conditions leading to timeouts in each terrain to identify patterns and potential causes.
2.  **Refine Landing Logic:** Re-evaluate the landing trigger and descent parameters in `drone_agent.py` and `adaptation_controller.c` to ensure consistent and timely landings.
3.  **Optimize Search Pattern:** Further optimize the spiral search pattern or introduce alternative search strategies to improve pad detection efficiency.
4.  **Increase Test Coverage:** Expand the benchmark to include more trials per terrain and a wider range of random seeds to thoroughly assess robustness.

Nathanael J. Bocker, 2026 all rights reserved
