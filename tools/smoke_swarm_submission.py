#!/usr/bin/env python3
"""Smoke-test a built Swarm CAE submission source folder.

Usage:
  python3 tools/smoke_swarm_submission.py build/swarm_cae_agent

This validates the submission-facing DroneFlightController without requiring the
Swarm CLI: import, init native CAE bridge, act() on a Swarm-shaped observation,
reset(), and action shape/range checks.
"""

from __future__ import annotations

import importlib
import pathlib
import sys
from typing import Sequence

import numpy as np


def _fail(msg: str) -> int:
    print(f"[FAIL] {msg}")
    return 1


def _make_observation() -> dict:
    # Template state layout:
    # position(3), rpy(3), linear velocity(3), angular velocity(3),
    # previous action(5), normalized altitude(1), relative search vector(3).
    position = np.array([18.0, 8.0, 12.0], dtype=np.float32)
    rpy = np.zeros(3, dtype=np.float32)
    vel = np.zeros(3, dtype=np.float32)
    rates = np.zeros(3, dtype=np.float32)
    prev_action = np.zeros(5, dtype=np.float32)
    altitude_norm = np.array([0.60], dtype=np.float32)
    search_vec = np.array([-12.0, -7.5, -11.8], dtype=np.float32)
    state = np.concatenate([position, rpy, vel, rates, prev_action, altitude_norm, search_vec]).astype(np.float32)

    # Mostly open scene with a shallow central obstacle patch to exercise depth ingestion.
    depth = np.ones((128, 128, 1), dtype=np.float32)
    depth[54:74, 58:70, 0] = 0.28
    return {"state": state, "depth": depth}


def main(argv: Sequence[str]) -> int:
    if len(argv) != 2:
        return _fail("Usage: python3 tools/smoke_swarm_submission.py build/swarm_cae_agent")

    source_dir = pathlib.Path(argv[1]).resolve()
    if not source_dir.exists():
        return _fail(f"Source folder does not exist: {source_dir}")
    if not (source_dir / "drone_agent.py").exists():
        return _fail(f"Missing drone_agent.py in {source_dir}")

    sys.path.insert(0, str(source_dir))

    try:
        module = importlib.import_module("drone_agent")
        Controller = getattr(module, "DroneFlightController")
        controller = Controller()
    except Exception as exc:
        return _fail(f"Could not import/init DroneFlightController: {exc}")

    obs = _make_observation()
    try:
        action = controller.act(obs)
    except Exception as exc:
        return _fail(f"act() raised: {exc}")

    action = np.asarray(action, dtype=np.float32).reshape(-1)
    if action.shape != (5,):
        return _fail(f"Expected action shape (5,), got {action.shape}")
    if not np.all(np.isfinite(action)):
        return _fail(f"Action contains non-finite values: {action}")
    if np.any(action[:3] < -1.0) or np.any(action[:3] > 1.0):
        return _fail(f"Direction components out of range: {action}")
    if not (0.0 <= float(action[3]) <= 1.0):
        return _fail(f"Speed component out of range: {action}")
    if not (-1.0 <= float(action[4]) <= 1.0):
        return _fail(f"Yaw component out of range: {action}")

    try:
        controller.reset()
        action2 = np.asarray(controller.act(obs), dtype=np.float32).reshape(-1)
    except Exception as exc:
        return _fail(f"reset()/second act() raised: {exc}")
    if action2.shape != (5,) or not np.all(np.isfinite(action2)):
        return _fail(f"Second action invalid: {action2}")

    if hasattr(controller, "engine") and hasattr(controller.engine, "close"):
        controller.engine.close()

    print("[PASS] Swarm CAE submission smoke test")
    print("action:", np.round(action, 4))
    print("post-reset action:", np.round(action2, 4))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
