# Constrained Adaptive Engine

**An MCU-oriented autonomy kernel for bounded drone control in sparse-perception environments.**

Constrained Adaptive Engine (CAE) is a compact, deterministic flight-control kernel designed for autonomous drone behavior under tight compute, memory, and safety constraints. Unlike conventional autonomous-drone stacks that depend on large perception models, GPU-class planning, or high-level semantic scene understanding, CAE focuses on a smaller control problem:

> Given limited onboard compute, sparse geometric perception, and strict safety bounds, what is the safest useful command the vehicle can issue right now?

The Python files in this repository are primarily validation harnesses. The core architecture is built around native C control and depth-processing paths that are intended to remain portable to MCU-class or resource-constrained embedded targets.

## Why This Is Different

Most autonomy systems are perception-first: detect objects, classify the scene, infer intent, then plan a path. CAE is control-first. It uses reduced depth geometry, platform state, and terrain-specific constraints to produce bounded motion commands without requiring a large neural perception stack.

Key differences:

- **MCU-first, not GPU-first**: native C control and depth-processing paths are kept separate from the Python simulator harness.
- **Bounded adaptation, not open-ended AI planning**: speed, descent, safety radius, attraction gain, and landing behavior are constrained by explicit profiles.
- **Sparse perception, not semantic perception**: CAE does not require object labels or scene graphs; it works from reduced geometric/depth cues.
- **Terrain-aware envelopes**: mountain, village, warehouse, forest, city, and open-valley cases each stress different safety behavior.
- **Verifiable behavior**: success, timeout, collision count, minimum distance, seed, and terrain are the primary validation signals.

## Current Verification Status

Stable baseline tag:

- `v0.1.0-safe-baseline`
- Baseline commit: `0bb38b0`
- Baseline benchmark: 5/6 terrain successes, 0 collisions, 1 safe Mountain timeout.

Active optimization branch:

- `feature/mountain-optimization`
- Focus: mountain corridor optimization, warehouse terminal commit guard, and local Swarm action-space compatibility.

## Key Features

- **JAX-free analytical core**: avoids JIT/runtime dependency overhead for deterministic low-latency control.
- **Native depth processing**: raw 128×128 depth images are reduced through `psmsl_depth_analyze_image()` into a 16×16 grid for C-side geometric analysis.
- **Adaptive breathing profiles**: safety margins and repulsion strength are scaled from depth-derived clutter and navigability metrics.
- **Terrain-specific landing policies**: mountain keeps a terrain-clearance floor before landing corridor descent; warehouse uses a guarded terminal commit path.
- **Local-minimum escape**: stall conditions activate escape logic to break potential-field symmetry.
- **Simulator validation harness**: `test_real_env.py`, `benchmark_subset.py`, and `benchmark_cae.py` test repeatable seeded scenarios.

## Project Structure

```text
constrained-adaptive-engine/
├── include/                         # C headers
├── src/                             # Native control and depth-processing code
├── docs/                            # Architecture, setup, and validation notes
├── tools/                           # Convenience scripts for local validation
├── constrained_adaptive_engine_bridge.py
├── test_real_env.py                 # Swarm integration test
├── benchmark_subset.py              # Focused terrain benchmark
├── benchmark_cae.py                 # Full terrain benchmark
└── README.md
```

## Build

Compile the native shared library on Linux/WSL:

```bash
gcc -O3 -shared -fPIC -Iinclude \
  -o libadaptive_controller.so \
  src/adaptation_controller.c \
  src/psmsl_depth_processor.c \
  -lm
```

Or use:

```bash
bash tools/build_native.sh
```

## Quick Validation

Run the ctypes bridge smoke test:

```bash
python3 constrained_adaptive_engine_bridge.py
```

Run the real-environment integration test:

```bash
python3 test_real_env.py
```

Run the focused terrain benchmark:

```bash
python3 benchmark_subset.py --terrains 3 4 5 6 --trials 1 --fixed
```

Run the full benchmark:

```bash
python3 benchmark_cae.py --trials 1 --fixed
```

## Local Swarm Compatibility

Some local Swarm/Crazyflow builds expose a 4-wide action space:

```text
[dir_x, dir_y, dir_z, speed]
```

Other validator-style builds may expose a 5-wide action space:

```text
[dir_x, dir_y, dir_z, speed, yaw]
```

CAE keeps the internal command representation canonical and adapts at the simulator boundary when needed. See `docs/local_swarm_setup.md` for WSL setup notes and dependency caveats.

## Depth Processing Architecture

The depth pipeline is designed to remain MCU-compliant:

1. `adapt_process_sensor_data()` receives a raw `float*` depth buffer.
2. `psmsl_depth_analyze_image()` reduces the 128×128 depth image to a 16×16 grid and computes geometric features.
3. `psmsl_depth_analyze_obstacles()` bins local obstacle evidence into spatial metrics.
4. Clutter density, local navigability, and collision-risk metrics modulate the adaptive control envelope.

## Documentation

- `docs/approach.md` — the core technical argument and positioning.
- `docs/mcu_portability.md` — why the architecture maps to MCU-class control.
- `docs/local_swarm_setup.md` — WSL/local Swarm setup notes.
- `docs/benchmarking.md` — benchmark commands and acceptance gates.

## Authorship

Copyright © 2026 Nathanael J. Bocker. All rights reserved.

## License and Commercial Use

This repository is provided under the **Constrained Adaptive Engine Non-Commercial Source-Available License**.

The code, documentation, algorithms, architecture, benchmarks, and related materials are available for internal non-commercial review, research, testing, and evaluation only.

Commercial use is prohibited without prior express written permission from Nathanael J. Bocker.

This includes, but is not limited to, product integration, paid pilots, customer demonstrations, hosted services, consulting deliverables, commercial benchmarking, derivative commercialization, sublicensing, redistribution, or use in a competing commercial implementation.

See `LICENSE` and `NOTICE.md`.
