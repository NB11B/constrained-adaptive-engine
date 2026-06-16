# Architecture Audit: Constrained Adaptive Engine & Swarm Subnet 124

## 1. Fundamentals: System Overview and Component Nodes

This document provides a comprehensive architectural map of the integration between the **Constrained Adaptive Engine** (C-based flight controller) and the **Swarm Subnet 124 Environment** (PyBullet simulation). 

The fundamental goal of this system is to navigate a drone from a spawn point to a landing platform across various procedurally generated terrains while avoiding obstacles and landing precisely.

### 1.1. Environment (swarm-subnet)
*   **MovingDroneAviary**: The core PyBullet environment class that simulates the drone physics, sensors, and scoring.
*   **MapTask**: Defines the procedural generation parameters for the 6 terrain types (City, Open, Mountain, Village, Warehouse, Forest).
*   **Validator/Reward**: The scoring mechanism (`flight_reward`) based on success, time, and safety.

### 1.2. Controller (constrained-adaptive-engine)
*   **DroneFlightController (`drone_agent.py`)**: The Python entry point required by the validator.
*   **CTypes Bridge (`constrained_adaptive_engine_bridge.py`)**: Interfaces Python with the compiled C library.
*   **Adaptation Controller (`adaptation_controller.c`)**: The core C engine implementing potential field navigation, state machines, and landing logic.
*   **PSMSL Depth Processor (`psmsl_depth_processor.c`)**: C-based depth image processing for obstacle detection and ground-plane filtering.

## 2. Data Flow and Interface Contracts

### 2.1. Observation Pipeline (Env → Controller)
1.  **PyBullet Simulation**: Generates raw depth buffer (128x128).
2.  **`_process_depth`**: Converts raw buffer to normalized depth map [0,1] for the 0.5-20m range.
3.  **`_computeObs`**: Constructs the observation dictionary:
    *   `depth`: (128, 128, 1) float32 [0,1]
    *   `state`: (116,) float32 vector
4.  **`DroneFlightController.act(obs)`**: Receives the observation dictionary.
5.  **CTypes Bridge**: Passes `depth` and unpacked `state` to the C engine via `adapt_process_sensor_data`.

### 2.2. State Vector Layout (116 Dimensions)
The exact layout of the state vector (verified at runtime with `ctrl_freq=50`, `action_dim=4`):
*   `[0:3]`: Drone Position (X, Y, Z) in meters.
*   `[3:6]`: Drone Orientation (Roll, Pitch, Yaw) in radians.
*   `[6:9]`: Drone Linear Velocity (Vx, Vy, Vz) in m/s.
*   `[9:12]`: Drone Angular Velocity (Roll_rate, Pitch_rate, Yaw_rate) in rad/s.
*   `[12:112]`: Action History (25 previous actions, each 4D).
*   `[112]`: Normalized AGL (Altitude Above Ground Level / `MAX_RAY_DISTANCE`).
*   `[113:116]`: Search Area Vector (Target Center - Drone Position).

### 2.3. Action Pipeline (Controller → Env)
1.  **C Engine `calculate_control_output`**: Computes desired velocity components (`vx`, `vy`, `vz`) and normalizes to `speed_norm`.
2.  **CTypes Bridge**: Returns a 5-element array `[vx, vy, vz, total_speed, yaw_cmd]`.
3.  **`DroneFlightController.act`**: Translates the 5-element C output into the required 5-element action array: `[dir_x, dir_y, dir_z, speed, yaw]`.
4.  **Validator RPC**: Receives the 5-element action.
5.  **`_preprocessAction` (VelocityAviary)**: Extracts the first 4 elements `[dir_x, dir_y, dir_z, speed]` to construct the target velocity vector. **Note: The 5th element (yaw) is ignored by the base `VelocityAviary` implementation.**
6.  **PyBullet Physics**: Applies RPMs to drone motors to achieve the target velocity.

## 3. Dynamic Values and Constants

*   `SPEED_LIMIT`: 3.0 m/s
*   `MAX_YAW_RATE`: 3.141 rad/s
*   `DEPTH_MIN_M`: 0.5 m
*   `DEPTH_MAX_M`: 20.0 m
*   `LANDING_PLATFORM_RADIUS`: 0.6 m
*   `LANDING_MAX_VZ`: 0.5 m/s
*   `LANDING_MAX_VXY_REL`: 0.6 m/s
*   `SIM_DT`: 0.02 s (50 Hz)
*   `MAX_TILT_RAD`: 1.047 rad (60 degrees)

## 4. Landing Fix Status

The current landing logic in `adaptation_controller.c` uses a target hover height (`land_target_z`) and a descent velocity (`vz_feather`). 

*   **Current Issue**: The drone reaches the pad but fails to make physical contact (normal force > 0.01 N).
*   **Proposed Fix (in C code, pending validation)**: 
    *   Set `land_target_z = tgt_z + 0.05f` (hover 5cm above pad).
    *   When `h_rem < 0.02f`, apply `vz_feather = -1.5f`.
    *   This translates to `speed_norm = 1.5 / 3.0 = 0.5`, resulting in a downward target velocity of -1.5 m/s, which should generate sufficient force for PyBullet contact detection.

## 5. Required `drone_agent.py` Implementation

Based on the audit, the `drone_agent.py` must:
1.  Implement `DroneFlightController` with `__init__`, `act(obs)`, and `reset()`.
2.  Initialize the C engine via the bridge.
3.  Unpack the 116-dim state vector (not 141 as previously assumed).
4.  Pass the `depth` image directly to the C engine (numpy unprojector is not needed as C engine handles depth natively).
5.  Return a 5-element action array `[dir_x, dir_y, dir_z, speed, yaw]`.

---
Nathanael J. Bocker, 2026 all rights reserved
