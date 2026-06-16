# Technical Approach

Constrained Adaptive Engine (CAE) is built around a control-first view of drone autonomy. The system does not attempt to reproduce a full robotics stack inside a small vehicle. Instead, it isolates the smallest useful autonomy kernel: a bounded controller that converts sparse state and depth evidence into safe motion commands.

## Core Thesis

Most autonomous-drone systems are perception-first. They begin with scene understanding, object detection, classification, mapping, or learned trajectory planning. CAE starts from a different premise:

> Autonomous behavior can be reduced to a deterministic adaptive control loop when the environment is represented as bounded geometric risk rather than semantic meaning.

This makes CAE suitable for resource-constrained platforms where GPU-class perception or large neural policies are not available, not desirable, or not dependable under degraded sensing.

## Control-First Stack

CAE separates autonomy into three layers:

1. **Perception reduction** — depth data is reduced into compact geometric metrics.
2. **Constraint shaping** — terrain-specific profiles define speed, altitude, safety, and landing envelopes.
3. **Bounded command generation** — the controller emits normalized motion commands constrained by the active envelope.

The result is a compact decision kernel rather than an open-ended planner.

## Sparse Perception

CAE does not require object labels, semantic segmentation, or a learned world model. The depth pipeline extracts local geometric structure and uses that information to shape the control response.

This is intentionally less expressive than a full perception stack, but it is more portable, more inspectable, and easier to test under deterministic conditions.

## Bounded Adaptation

Adaptation in CAE is not unconstrained. The controller adapts inside explicit bounds:

- maximum speed
- safety radius
- attraction gain
- descent rate
- terminal landing thresholds
- terrain-specific clearance floor
- warehouse terminal commit criteria

This creates a system that can change behavior in response to the environment while remaining explainable and auditable.

## Terrain Profiles

The current validation harness uses terrain profiles because each environment stresses a different failure mode:

- **Mountain**: ridge clearance, long approach corridor, controlled descent.
- **Village**: dense clutter and lateral navigation.
- **Warehouse**: precision landing, pad-edge risk, terminal commitment.
- **Forest**: cluttered approach and touchdown under occlusion.
- **City/Open Valley**: baseline obstacle and landing behavior.

The profiles are not a weakness of the design. They are the mechanism that turns the same native kernel into environment-specific safe behavior.

## Simulator Boundary

CAE keeps its internal command representation independent from local simulator quirks. Some local Swarm builds use a 4-wide action space:

```text
[dir_x, dir_y, dir_z, speed]
```

Other validator-style builds may expose a 5-wide action space:

```text
[dir_x, dir_y, dir_z, speed, yaw]
```

The correct compatibility model is to preserve CAE's internal command logic and adapt only at the simulator boundary.

## What Success Means

A successful CAE run is not just a visually convincing trajectory. It should be measured by repeatable, seeded metrics:

- success or timeout
- collision count
- minimum distance to target
- minimum terrain clearance
- landing phase behavior
- terrain type
- seed

This makes the repository suitable for engineering review rather than demo-only presentation.
