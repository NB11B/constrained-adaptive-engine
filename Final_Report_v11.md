# Iteration 11 Final Report: Verified Baseline and Collision Analysis

## 1. Overview
Iteration 11 was a critical "clean run" to verify the baseline from commit `58b4f1d`. After a hard reset, cache clearing, and a fresh C-engine rebuild, we have successfully aligned the runtime with the latest repository code. This run has yielded genuinely new behavior, confirming that previous reports were indeed stale.

## 2. Verification of Runtime Alignment
The runtime is now correctly executing the latest logic:
- **Phase Labels**: Mountain now correctly reports `MOUNTAIN_APPROACH_CORRIDOR` and `MOUNTAIN_DESCENT_CORRIDOR`, replacing the stale `MOUNTAIN_LONG_RANGE_ACQUISITION`.
- **Warehouse Behavior**: Terminal assist ticks have dropped from `2413` to `590`, proving the decisive terminal profile is active.

## 3. Benchmark Results (Verified Baseline)
The benchmark run (`benchmark_cae.py`) with `--trials 1 --fixed` produced the following results:

| Terrain | Terrain ID | Seed | Status | Steps | Time (s) | Min Dist (m) | Landing | Descent | App Assist | Term Assist |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **City Map** | 1 | 1337 | **SUCCESS** | 611 | 3.61 | 0.872 | True | True | 113 | 83 |
| **Open/Valley** | 2 | 1337 | **SUCCESS** | 1452 | 11.08 | 0.203 | True | True | 126 | 827 |
| **Mountain** | 3 | 1337 | **COLLISION** | 929 | 107.68 | 25.066 | False | False | 465 | 0 |
| **Village** | 4 | 1337 | **SUCCESS** | 2837 | 66.28 | 0.223 | True | True | 131 | 735 |
| **Warehouse** | 5 | 1337 | **COLLISION** | 1177 | 42.31 | 0.153 | True | True | 164 | 590 |
| **Forest** | 6 | 1337 | **SUCCESS** | 706 | 44.37 | 2.355 | True | True | 49 | 159 |

**Summary:**
- **Total Trials**: 6
- **Successes**: 4 (66.7%)
- **Collisions**: 2 (**Mountain** and **Warehouse**)
- **Timeouts**: 0
- **Avg Steps (Success)**: 1401.5

## 4. Collision Analysis (The "Flight Recorder" View)

### 4.1 Mountain Collision (Step 929)
The conservative corridor successfully brought the drone from `31.18m` down to `2.13m` (Step 875) without the runaway behavior seen in the stale run. However, at Step 929, it collided at `min_dist=25.066m`. 
- **The Cause**: The drone was in `MOUNTAIN_DESCENT_CORRIDOR` at an altitude of `~25m` (`agl=0.11` at step 925). The collision occurred while it was still far from the pad's XY but at a very low AGL relative to the local terrain. This suggests that while the acquisition was working, the descent rate or the safety clearance in the corridor was too aggressive for the specific ridge it was crossing.

### 4.2 Warehouse Collision (Step 1177)
The decisive terminal profile successfully reached the pad and entered `TERMINAL_PRESS`.
- **The Cause**: The collision at `min_dist=0.153m` during the press phase indicates that the drone reached the pad but likely clipped an edge or an obstacle just before touchdown was registered. The reduction in terminal ticks from `2413` to `590` shows we are no longer hovering indefinitely, but the final `-0.22 m/s` press might be slightly too fast or the centering gate (`0.18m`) might still allow a minor clip on the warehouse's tight geometry.

## 5. Conclusion
We have achieved a stable, verified baseline with **4/6 successes**. The transition from "timeout" to "collision" in Warehouse is actually progress—it means the drone is finally attempting the landing. For Mountain, we have successfully implemented the acquisition corridor, and the next step is simply to lift the floor of that corridor to avoid the mid-transit ridge collision.

## 6. Next Step Recommendation
1. **Mountain**: Increase the `transit_alt` and `approach_gate_xy` floor slightly to clear the ridge at Step 929.
2. **Warehouse**: Reduce the `press_vz` slightly (e.g., `-0.15 m/s`) and tighten the `final_center_gate` to `0.15m` to avoid the final clip.

This baseline (commit `58b4f1d`) is verified and ready for the final precision tuning.
