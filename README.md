# Constrained Adaptive Engine

The **Constrained Adaptive Engine** is a high-performance, JAX-free flight controller designed for Subnet 124. It is repurposed from the `adaptive-engine` framework and optimized for MCU-compliant execution (e.g., ESP32) with zero JIT runtime overhead.

## Technical Architecture

The engine utilizes a **JAX-Free Analytical Core** that implements a deterministic Hamiltonian Potential Field and landing state machine in pure C. This architecture eliminates the non-deterministic latency spikes associated with JIT compilation, ensuring **Deterministic Performance** with a guaranteed sub-2ms execution time per step. This performance profile is well within the 20ms validator-side timeout boundary required by Subnet 124.

Furthermore, the system incorporates **PSMSL-Inspired Depth Processing**, which adapts the "phi scaled mirrored semantic logic" (PSMSL) framework to analyze 128x128 depth maps. This process extracts high-level navigation metrics, such as clutter density, local navigability, and collision risk, providing a more efficient way to navigate dense clutter. The entire engine is **MCU Compliant**, designed for bare-metal or RTOS execution with a minimal memory footprint, and includes a **Production-Ready Bridge** for seamless Python integration.

## Repository Organization

The project is structured to separate the core C implementation from the Python integration and verification layers. The `src/` directory contains the primary C source files, including `adaptation_controller.c` and `psmsl_depth_processor.c`, while the `include/` directory houses the corresponding header files. The `constrained_adaptive_engine_bridge.py` script provides the `ctypes` interface, and `evaluate_constrained_engine.py` serves as the performance verification harness for the `crazyflow-swarm-sim` environment. The compiled `libadaptive_controller.so` shared library is also included for immediate deployment on Linux systems.

## Verification Results

The engine has been verified across all six Subnet 124 terrains (City, Open/Valley, Mountain, Village, Warehouse, Forest) using the `evaluate_constrained_engine.py` script, achieving high success rates with minimal latency.

---
*Copyright © 2026 SOTAPilot Development Team. All rights reserved.*
