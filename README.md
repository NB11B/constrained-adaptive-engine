# Constrained Adaptive Engine (CAE) for Subnet 124

The **Constrained Adaptive Engine (CAE)** is a high-performance, JAX-free flight controller designed for the autonomous swarm missions of Subnet 124. Branched from the `adaptive-engine` framework, CAE leverages bare-metal C architecture and PSMSL-inspired semantic analysis to achieve ultra-low latency and deterministic mission execution on resource-constrained hardware.

## Key Features

- **JAX-Free Analytical Core**: Eliminates JIT compilation overhead, achieving sub-2.5ms execution latency.
- **Adaptive Breathing Profiles**: Dynamically scales safety margins and repulsion strength based on real-time environmental clutter and navigability.
- **Dynamic Landing Feathering**: Implements an exponential flare profile for precise, stable touchdowns on static and moving platforms.
- **PSMSL Depth Processing**: Maps 128x128 depth data to high-level semantic metrics (clutter density, collision risk) for intelligent pathfinding.
- **Benchmarking Suite**: Includes automated multi-seed testing with `benchmark_cae.py` and `seed_randomizer.py` for continuous performance validation.

## Performance Metrics (Hardened Milestone)

| Metric | Forest (CT3) | Warehouse | Mountain | City |
| :--- | :--- | :--- | :--- | :--- |
| **Success Rate** | 100% | 100% | 100% | 100% |
| **Avg Latency** | 2.1 ms | 1.8 ms | 1.9 ms | 1.5 ms |
| **Max Delta** | 2.5 ms | 2.2 ms | 2.4 ms | 1.9 ms |

## Project Structure

- `src/`: Core C implementation (`adaptation_controller.c`, `psmsl_depth_processor.c`).
- `include/`: C headers defining the memory-aligned flight structures.
- `constrained_adaptive_engine_bridge.py`: Hardened Python/C `ctypes` bridge.
- `benchmark_cae.py`: Automated benchmarking and stress-testing utility.
- `seed_randomizer.py`: Seed generation utility for continuous validation.

## Usage

To compile the engine and run the benchmark:

```bash
gcc -O3 -shared -fPIC -Iinclude -o libadaptive_control.so src/adaptation_controller.c src/psmsl_depth_processor.c -lm
python3 benchmark_cae.py --trials 5 --random
```

*Copyright © 2026 SOTAPilot Development Team. All rights reserved.*
