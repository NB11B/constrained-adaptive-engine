# MCU Portability Notes

CAE is designed so the autonomy-critical path can be moved toward embedded hardware without bringing the entire simulator or Python harness with it.

## What Is MCU-Oriented Here

The MCU-oriented portion is the native control/depth path, not the Python validation environment. The intended split is:

- **C layer**: deterministic control, depth reduction, obstacle metrics, bounded command generation.
- **Python layer**: simulator integration, ctypes bridge, benchmark orchestration, result collection.

This distinction matters. CAE is not claiming that a full simulator or robotics stack runs on an MCU. It is claiming that the decision kernel can be reduced to a small native component that can be tested in simulation and then ported downward.

## Design Constraints

The embedded-facing design should preserve these constraints:

- fixed-size buffers
- no Python dependency in the flight-critical loop
- bounded command output
- explicit speed/altitude/safety limits
- deterministic state-machine transitions
- small depth representation after reduction
- no neural inference requirement in the core control path

## Depth Reduction

The depth image begins as a simulator-scale frame, but the control path reduces it into a compact grid and derived metrics. This gives the controller enough geometry to respond to local risk without requiring semantic perception.

A practical embedded deployment could replace the simulator depth source with:

- a small depth camera
- stereo-derived range bins
- time-of-flight range slices
- optical-flow/range-fusion approximations
- preprocessed depth from a companion sensor

The control kernel should not depend on the original sensor format as long as equivalent local risk metrics are produced.

## Command Surface

The controller emits a compact command surface rather than a full trajectory:

```text
direction vector + normalized speed + optional yaw
```

This makes the engine suitable as either:

1. a direct low-level autonomy kernel, or
2. a safety/constraint layer under a larger planner.

## Why This Matters

Most autonomy stacks become expensive because they preserve too much information for too long: full frames, full maps, object classes, learned latent states, and high-dimensional planners. CAE deliberately collapses the problem early into bounded control variables.

That tradeoff reduces expressiveness, but it improves portability, inspectability, and repeatable validation.

## Porting Checklist

Before moving the kernel to a target MCU or embedded board, confirm:

- native library builds cleanly with target compiler
- depth buffers are fixed-size or statically bounded
- all allocations in the flight-critical path are controlled
- timing is measured under target clock and memory limits
- command outputs are range-clamped
- watchdog/failsafe behavior is explicit
- simulator-only assumptions are removed from the C path

## Non-Goals

CAE is not intended to replace every part of an autonomous-drone stack. It does not attempt to provide:

- global SLAM
- semantic object recognition
- learned policy training
- multi-agent strategic planning
- long-horizon mission planning

Its role is the bounded local autonomy layer: compact enough to reason about, strict enough to test, and small enough to move toward embedded deployment.
