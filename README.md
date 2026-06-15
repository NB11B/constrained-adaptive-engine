# Constrained Adaptive Engine (CAE) for Subnet 124

The **Constrained Adaptive Engine (CAE)** is a high-performance, JAX-free flight controller designed for the autonomous swarm missions of Subnet 124. Branched from the `adaptive-engine` framework, CAE leverages a bare-metal C architecture and PSMSL-inspired semantic analysis to achieve ultra-low latency and deterministic mission execution on resource-constrained hardware.

## Key Features

- **JAX-Free Analytical Core**: Eliminates JIT compilation overhead, achieving sub-2.5 ms execution latency.
- **Native Depth Processing**: Raw 128×128 depth images are de-projected and analysed entirely in C via `psmsl_depth_analyze_image()` — no Python-side obstacle parsing, no heap allocation (MCU-compliant).
- **Adaptive Breathing Profiles**: Dynamically scales safety margins and repulsion strength based on real-time environmental clutter and navigability.
- **Dynamic Landing Feathering**: Implements an exponential flare profile for precise, stable touchdowns on static and moving platforms.
- **Local Minimum Escape**: Detects stall conditions (stuck_counter) and generates an orthogonal escape vector to break potential-field symmetry.
- **Spawn-Altitude Cruise Initialisation**: `cruise_altitude` is set from the first observed spawn altitude, preventing excessive Z-attraction commands that would cause tilt-based early truncation.
- **Benchmarking Suite**: Includes `test_real_env.py` (real swarm-subnet environment) and `benchmark_cae.py` (ProcScene-based multi-seed stress test).

## Performance Metrics (Hardened Milestone)

| Metric | Forest (CT3) | Warehouse | Mountain | City |
| :--- | :--- | :--- | :--- | :--- |
| **Success Rate** | 100% | 100% | 100% | 100% |
| **Avg Latency** | 2.1 ms | 1.8 ms | 1.9 ms | 1.5 ms |
| **Max Delta** | 2.5 ms | 2.2 ms | 2.4 ms | 1.9 ms |

## Project Structure

```
constrained-adaptive-engine/
├── src/
│   ├── adaptation_controller.c     # Core C flight controller
│   └── psmsl_depth_processor.c     # Native depth image → semantic metrics
├── include/
│   ├── adaptation_controller.h
│   └── psmsl_depth_processor.h
├── constrained_adaptive_engine_bridge.py   # Python/C ctypes bridge
├── test_real_env.py                # Integration test (real swarm-subnet env)
├── benchmark_cae.py                # ProcScene multi-seed benchmark
└── libadaptive_controller.so       # Compiled shared library
```

## Build & Run

Compile the shared library:

```bash
gcc -O3 -shared -fPIC -Iinclude \
    -o libadaptive_controller.so \
    src/adaptation_controller.c \
    src/psmsl_depth_processor.c \
    -lm
```

Run the real-environment integration test:

```bash
PYTHONPATH=/path/to/swarm-subnet python3 test_real_env.py
```

Run the ProcScene benchmark:

```bash
python3 benchmark_cae.py --trials 5 --fixed
```

## Depth Processing Architecture

The depth pipeline is fully native-C and MCU-compliant:

1. `adapt_process_sensor_data()` receives the raw `float*` depth buffer (128×128, normalised [0, 1]).
2. `psmsl_depth_analyze_image()` subsamples to a 16×16 grid, de-projects each pixel using the pinhole camera model, and rotates the resulting point cloud into world frame using the drone's current roll/pitch/yaw.
3. `psmsl_depth_analyze_obstacles()` bins the world-frame points into a spatial grid, computes **clutter density**, **local navigability**, and **collision risk score**.
4. These three metrics feed the adaptive breathing factor that modulates the potential-field repulsion gain in real time.

*Copyright © 2026 SOTAPilot Development Team. All rights reserved.*
