# Iteration 9 Final Report: Final Acquisition Corridor and Terminal Landing Baseline

## 1. Overview
Iteration 9 focused on finalizing the acquisition corridor for Mountain terrain and tightening the terminal landing baseline for Warehouse. This involved implementing a working baseline patch with deterministic terrain profiles, bounded Mountain behavior, late Warehouse terminal assist, and telemetry that explains which policy branch is active.

## 2. Changes Implemented
- **Mountain Long-Range Acquisition**: Implemented a true long-range acquisition corridor for Mountain with `approach_gate_xy: 35.0`, `acq_gain: 0.92`, and `acq_speed_max: 2.05`. Explicit phase handling (`MOUNTAIN_LONG_RANGE_ACQUISITION`, `MOUNTAIN_SAFETY_CLIMB`, `MOUNTAIN_DESCENT_CORRIDOR`) was added.
- **Warehouse Terminal Assist**: Tightened terminal activation with `term_gate_xy: 0.25`, `final_center_gate: 0.11`, `settle_required: 45`, and `press_vz: -0.10` to ensure later engagement, avoid long hovering, and enable more decisive pressing.
- **City/Village/Forest Approach Gates**: Added moderate `approach_gate_xy` values for these terrains to improve close-but-timeout scenarios without affecting Mountain/Warehouse special handling.
- **Telemetry**: Confirmed that telemetry now labels Python-side assist phases correctly, including `MOUNTAIN_SAFETY_CLIMB`, `ACQUISITION_ASSIST`, `SETTLE_PHASE`, and `TERMINAL_PRESS`.

## 3. Validation Results (Iteration 9 Baseline)
The validation run (`test_real_env.py`) showed the following:

| Terrain | Seed | Status | Steps | Min Dist | Landing | Descent | Assist (App/Term) | Time (s) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Mountain** | 1337 | **TIMEOUT** | 146 | 40.15m | False | False | 146 / 0 | 12.82 |
| **Warehouse** | 1337 | **TIMEOUT** | 3000 | 0.13m | True | True | 135 / 2413 | 111.78 |
| **Open/Valley** | 39259 | **SUCCESS** | 1126 | 0.217m | True | True | 6 / 699 | 8.95 |

### 3.1 Analysis of Mountain Timeout
The Mountain terrain still results in a **TIMEOUT** with a large `min_dist` (40.15m). The `landing=False` and `descent=False` flags indicate that the drone is not entering the C-engine's landing state. This suggests that despite the long-range acquisition corridor, the drone is still struggling to navigate the complex terrain and reach a position where the landing sequence can be initiated within the time limit.

### 3.2 Analysis of Warehouse Timeout
The Warehouse terrain resulted in a **TIMEOUT** with an extremely low `min_dist` (0.13m). While the `landing=True` and `descent=True` flags confirm that the drone is entering the landing and descent phases, the timeout indicates that the final touchdown criteria are not being met within the allotted time. The high `terminal_assist_ticks` (2413) suggests that the drone is spending a significant amount of time in the terminal phase, potentially hovering just above the pad without fully settling.

## 4. Benchmark Results (Iteration 9 Baseline)
The benchmark run (`benchmark_cae.py`) with `--trials 1 --fixed` yielded the following:

| Terrain | Terrain ID | Seed | Status | Steps | Time (s) | Min Dist (m) | Landing Phase | Descent Phase | Approach Assist Ticks | Terminal Assist Ticks |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **City Map** | 1 | 1337 | **SUCCESS** | 618 | 3.89 | 0.824 | True | True | 113 | 87 |
| **Open/Valley** | 2 | 1337 | **SUCCESS** | 1416 | 11.41 | 0.158 | True | True | 104 | 829 |
| **Mountain** | 3 | 1337 | **TIMEOUT** | 146 | 18.69 | 40.150 | False | False | 146 | 0 |
| **Village** | 4 | 1337 | **SUCCESS** | 2719 | 69.99 | 0.227 | True | True | 131 | 615 |
| **Warehouse** | 5 | 1337 | **TIMEOUT** | 3000 | 117.21 | 0.130 | True | True | 135 | 2413 |
| **Forest** | 6 | 1337 | **SUCCESS** | 679 | 47.63 | 2.357 | True | True | 41 | 139 |

**Summary:**
- **Total Trials**: 6
- **Successes**: 4 (66.7%)
- **Collisions**: 0
- **Timeouts**: 2
- **Landing Seen**: 5
- **Approach Assist Seen**: 6
- **Terminal Assist Seen**: 5
- **Descent Seen**: 5
- **Avg Steps (Success)**: 1358.0
- **Avg Wall Time**: 33.23s

## 5. Next Steps
- **Mountain**: Further investigate the long-range acquisition strategy. The drone is still timing out far from the pad. This might require a more aggressive navigation strategy or a re-evaluation of the `acq_gain` and `acq_speed_max` parameters within the `MOUNTAIN_LONG_RANGE_ACQUISITION` phase.
- **Warehouse**: Focus on the final touchdown criteria. The drone is getting very close but timing out. This suggests that the `final_center_gate`, `settle_required`, or `press_vz` parameters need fine-tuning to ensure a decisive and timely touchdown.
- **Overall**: Consider increasing the `max_steps` for the benchmark or adjusting the `time_limit` to allow more time for complex terrains, especially if the current timeouts are due to insufficient time rather than fundamental navigation issues.

## 6. Repository Status
All changes for Iteration 9, including the refined `test_real_env.py` and `flight_recorder.py`, have been committed to the `master` branch with commit `4fddf0304b4ebec3afd4c70b19d76acb063472f3`.
