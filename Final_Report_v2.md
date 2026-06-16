# Final Report: Adaptive Engine for Drone Swarms (Subnet 124) - Iteration 2

## Introduction
This report details the second iteration of updates to the adaptive engine for drone swarms, focusing on improving the autonomous landing policy for the Swarm Subnet 124 challenge. The previous iteration highlighted issues with consistent and timely landings, primarily manifesting as timeouts and collisions. This report outlines the new implemented changes, the validation process, and the updated benchmarking results.

## Implemented Changes
Significant modifications were made to the drone's landing policy and search patterns, as well as tuning of the C-engine controller:

### `drone_agent.py`
*   **Committed Landing Policy:** The agent can now commit to landing through three paths:
    *   Confirmed AGL/low-pass detection.
    *   Repeated near-target dwell.
    *   Sustained low-altitude search-zone dwell.
*   New policy constants were introduced:
    ```python
    SEARCH_ZONE_M = 5.0
    COMMIT_ZONE_M = 1.35
    LOW_PASS_AGL_M = 1.80
    SEARCH_DWELL_TICKS = 180
    COMMIT_DWELL_TICKS = 35
    ```
*   The agent now tracks `_landing_committed`, `_near_target_ticks`, and `_search_zone_ticks`, calling `_commit_landing()` when sufficient evidence for descent is gathered.
*   **Faster and Deterministic Search Pattern:** The gradual spiral search was replaced with a cross/square sweep around the noisy target to force more pad overflights within the time limit.
*   **Aggressive Landing Parameters:** Once landing is committed, safety repulsion is lowered, and target authority is increased:
    ```python
    safety_radius = 0.80 if committed else 1.10
    mode_v = 80.0 if committed else 160.0
    attraction_gain = 14.0 if committed else 11.0
    landing_threshold_xy = 0.65
    landing_descent_rate = 0.45
    ```

### `adaptation_controller.c`
*   **Retuned Landing State Machine:** The C controller now enters landing mode at a wider gate (`dist_xy_lock < 3.0f`) and initiates descent when either the XY gate or low-pass AGL condition is met.
*   **Stronger Descent Authority:** The controller clamps the landing descent rate between `0.28f` and `0.80f`, uses faster downward targets, and provides 8x vertical acceleration authority to expedite descent.
*   **Ignored Near-Pad Obstacle Repulsion:** During landing/descent, the controller increases the pad obstacle ignore radius to prevent projected depth points near the landing target from pushing the drone away during final approach.

## Validation Results (`test_real_env.py`)
The `test_real_env.py` script was executed after the latest changes. The log indicated a `TIMEOUT` at step 2999, with a minimum distance of 2.704m. While still a timeout, the minimum distance has slightly improved compared to the previous iteration (from 3.374m to 2.704m), suggesting the drone is getting closer to the target before timing out.

```
TERMINATED at step=2999 dist=3.55 term=False trunc=True
=> TIMEOUT | steps=3000 | min_dist=2.704m | time=31.95s
```

## Benchmark Results (`benchmark_cae.py`)
The `benchmark_cae.py` script was run with `--trials 1 --fixed` to evaluate the performance across different terrains using a fixed seed (1337). The results are summarized below:

| Terrain     | Terrain ID | Seed | Status    | Steps | Time (s) | Min Dist (m) |
|-------------|------------|------|-----------|-------|----------|--------------|
| City Map    | 1          | 1337 | TIMEOUT   | 3000  | 10.80    | 13.006       |
| Open/Valley | 2          | 1337 | TIMEOUT   | 3000  | 22.98    | 0.536        |
| Mountain    | 3          | 1337 | COLLISION | 796   | 95.02    | 7.194        |
| Village     | 4          | 1337 | TIMEOUT   | 3000  | 56.43    | 73.844       |
| Warehouse   | 5          | 1337 | TIMEOUT   | 3000  | 100.14   | 3.497        |
| Forest      | 6          | 1337 | TIMEOUT   | 3000  | 199.77   | 0.537        |

**Summary:**
*   **Total Trials:** 6
*   **Successes:** 0 (0.0%)
*   **Collisions:** 1
*   **Timeouts:** 5

Comparing these results to the previous iteration, the overall outcome remains similar: no successful landings, a single collision, and multiple timeouts. While the minimum distance for some terrains (e.g., Open/Valley, Forest) is still very low, indicating proximity to the target, the drone is not consistently completing the landing sequence. The `City Map` and `Village` terrains show significantly higher minimum distances, suggesting the drone might be struggling to even approach the target effectively in these environments.

## Conclusion and Next Steps
The second iteration of changes aimed to address the timeout issues and improve landing decisiveness. While the `test_real_env.py` showed a slight improvement in minimum distance, the overall benchmark results indicate that the drone is still failing to achieve successful landings. The new committed landing policy and more aggressive landing parameters have not yet yielded the desired breakthrough.

**Next Steps:**
1.  **Detailed Log Analysis:** Conduct a deeper analysis of the `validation_new.log` and `benchmark_new.log` files to understand the drone's behavior leading up to timeouts and collisions in each terrain. Specifically, focus on the state of `_landing_committed`, `_near_target_ticks`, and `_search_zone_ticks` to verify if the new policy is being triggered as expected.
2.  **Visualize Trajectories:** If possible, visualize the drone's trajectories and target positions in the simulator for failed trials to identify any systematic errors in navigation or landing approach.
3.  **Parameter Tuning:** Further fine-tune the new policy constants (`SEARCH_ZONE_M`, `COMMIT_ZONE_M`, `LOW_PASS_AGL_M`, `SEARCH_DWELL_TICKS`, `COMMIT_DWELL_TICKS`) and landing parameters (`safety_radius`, `mode_v`, `attraction_gain`, `landing_threshold_xy`, `landing_descent_rate`) based on the detailed log analysis.
4.  **Refine Search Pattern:** Re-evaluate the effectiveness of the cross/square sweep search pattern. Consider adaptive search strategies that can adjust based on environmental feedback.
5.  **C-Engine Interaction:** Verify the interaction between the Python agent and the C engine, ensuring that the C-engine's landing state machine is being correctly engaged and executed.

Nathanael J. Bocker, 2026 all rights reserved
