# Flight Phases Implementation Review: Constrained Adaptive Engine

## 1. Executive Summary

This document cross-references the theoretical flight phases outlined in the `FLIGHT_PHASES_REFERENCE.md` against the actual C-based implementation of the Constrained Adaptive Engine (CAE) for Subnet 124. The audit evaluates how well the physics, control objectives, and human-pilot training lessons are embodied in the codebase, specifically focusing on `adaptation_controller.c` and `psmsl_depth_processor.c`.

Overall, the CAE successfully implements a staged, state-machine approach that closely mirrors human helicopter training, particularly in its handling of the approach and landing phases. The engine correctly treats the drone as a particle interacting with a potential field, while enforcing strict kinematic limits (e.g., velocity clamping and smoothing) to maintain the "dynamic equilibrium" required for stable flight.

## 2. Phase-by-Phase Code Audit

### 2.1. Takeoff and Hover Transition

**Theoretical Objective:** Vertical ascent, stabilization, overcoming ground effect, and counteracting torque.
**Code Implementation:** 
The takeoff phase is handled implicitly through the initialization of the engine and the `cruise_altitude` parameter. 
- In `drone_agent.py`, the engine is initialized with `cruise_altitude = self._spawn_z` (lines 105, 146).
- In `adaptation_controller.c` (line 175), the Z-attraction logic sets `target_z_ref = ctrl->params.cruise_altitude`.
- A proportional-derivative (PD) style controller applies a vertical velocity command (`desired_vz += ctrl->params.attraction_gain * 2.0f * dz_attraction`) to lift the drone from the ground to the hover altitude.
- The "torque counteract" is abstracted away by the PyBullet `VelocityAviary` environment, which accepts a 4-element velocity vector, but the CAE correctly outputs a 5-element vector including a calculated `yaw_cmd` (lines 378-381) to maintain a forward-facing heading toward the target.

### 2.2. Transition to Forward Flight and Cruise

**Theoretical Objective:** Pushing through flapback, managing altitude as Effective Translational Lift (ETL) occurs, and maintaining dynamic equilibrium.
**Code Implementation:**
The cruise phase is governed by the potential field gradient and strict velocity clamping.
- **Potential Field Navigation:** The `calculate_potential_gradient` function (lines 31-112) generates the primary movement vector based on attraction to the goal and repulsion from obstacles.
- **Velocity Management:** The engine clamps the maximum total speed to `SPEED_LIMIT` (3.0 m/s) during cruise (line 318).
- **Acceleration Smoothing (Anti-Flapback):** To prevent the drone from pitching aggressively (which causes altitude loss and instability in multirotors), the engine implements a strict velocity smoother (lines 338-358). It limits the change in velocity (`max_dv = 1.5f * SIM_DT`) to ensure smooth transitions, effectively mimicking a pilot "pushing through" transition smoothly rather than jerking the cyclic.
- **Dynamic Breathing:** The cruise phase incorporates the PSMSL depth processor's semantic metrics. The `breathing_factor` (lines 41-56) expands safety margins in high-clutter environments and contracts them in open spaces, mimicking a human pilot's increased caution in tight spaces.

### 2.3. Descent and Approach

**Theoretical Objective:** Establishing a stable glide path, arresting descent rate, and coordinating vertical/horizontal velocities to arrive at the spot with zero drift.
**Code Implementation:**
The CAE implements a highly explicit, multi-stage state machine for the approach, directly mirroring the helicopter pilot's need to coordinate cyclic (XY speed) and collective (Z speed).
- **Approach Trigger:** The `landing_phase` is triggered when `dist_xy_lock < 8.0f` (line 189).
- **Horizontal Damping:** During the approach, horizontal speed is linearly scaled down based on distance (`speed_xy = fminf(2.0f, fmaxf(0.15f, dist_xy_lock * 1.5f))` at line 307), and a damping factor is applied (line 322) to arrest momentum.
- **Descent Gate:** The engine refuses to begin vertical descent until the drone is physically over the pad. The `descent_phase` is only triggered when `dist_xy_lock < ctrl->params.landing_threshold_xy` (0.6m) and horizontal relative velocity is low (lines 215-221). This perfectly matches the training lesson of arriving at a stable hover *before* descending.

### 2.4. Landing, Rollout, and the "Autorotation Flare" Analogy

**Theoretical Objective:** Minimizing vertical velocity at touchdown, ensuring alignment, and (in autorotation) preserving rotor energy until the last moment.
**Code Implementation:**
The recent bug fix in the C engine perfectly encapsulates the lessons of the autorotation flare.
- **The Feathering Profile:** The `descent_phase` logic (lines 245-263) implements a stepped descent profile. It drops at `-0.8 m/s` in the mid-approach, slows to `-0.3 m/s` in the final approach, and finally "presses" at `-0.25 m/s` at pad level.
- **The "Flare" Fix:** Previously, the engine pressed at `-1.5 m/s` and allowed the drone to sink to `tgt_z - 0.10m`. This caused the drone to punch through the pad, much like a pilot failing to flare and crashing hard. The fix raised the floor to `tgt_z + 0.01f` and reduced the press velocity to `-0.25 m/s` (line 253). This ensures the drone touches down gently and stops pressing immediately upon contact, perfectly satisfying the `LANDING_MAX_VZ = 0.5 m/s` constraint.
- **Station-Keeping:** During descent, the potential field is bypassed for XY control. Instead, a direct proportional centering hold (`vx_center = dx_tgt * kxy`) is used (lines 273-291) to ensure the drone drops straight down without drifting off the pad.

## 3. Sensor Integration and Perception

**Theoretical Objective:** Fusing IMU, altimeter, and vision data for state estimation and obstacle avoidance.
**Code Implementation:**
The CAE uses a sophisticated, MCU-optimized perception pipeline.
- **State Ingestion:** `drone_agent.py` extracts the 116-dimensional state vector (pos, vel, rpy, agl) and feeds it to the C engine via the ctypes bridge (`process_sensor_data`).
- **PSMSL Depth Processing:** The raw 128x128 depth image is not processed in Python. It is passed as a contiguous C-array to `psmsl_depth_processor.c`.
- **World-Frame Mapping:** The C code subsamples the image, deprojects it using a pinhole camera model, and rotates the points into the world frame using the drone's current roll, pitch, and yaw (lines 162-213 in `psmsl_depth_processor.c`).
- **Semantic Extraction:** The points are fed into a 3D spatial grid to extract `clutter_density`, `local_navigability`, and `collision_risk_score`. These metrics directly feed the "breathing" logic in the flight controller, linking perception directly to the physics engine's safety margins.

## 4. Conclusion

The Constrained Adaptive Engine is a highly literal translation of aviation physics and pilot training into C code. The explicit separation of horizontal approach (`landing_phase`) from vertical drop (`descent_phase`), the strict velocity smoothing to prevent multirotor tilt, and the native 3D depth perception pipeline demonstrate a robust, aerospace-grade architectural design. The recent landing fix correctly applied the concept of "arresting descent rate" at the precise moment of touchdown, resulting in successful, compliant landings across all terrain types.
