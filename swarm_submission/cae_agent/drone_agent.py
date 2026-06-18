"""
Swarm Subnet submission adapter for Constrained Adaptive Engine (CAE).

This file is intended to be packaged as Swarm's required `drone_agent.py`.
It preserves the working branch's native C control path and terrain-aware
assist logic, but adapts it to Swarm's submission interface:

    class DroneFlightController:
        def act(self, observation) -> np.ndarray shape (5,)
        def reset(self) -> None

Expected packaged files beside this module:
- constrained_adaptive_engine_bridge.py
- src/adaptation_controller.c
- src/psmsl_depth_processor.c
- include/*.h

The bridge will load `libadaptive_controller.so` if present or compile it from
`src/` and `include/` when the validator container has a compiler available.
"""

from __future__ import annotations

import math
from typing import Dict, Optional, Tuple

import numpy as np

from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine


CAMERA_FOV_DEG = 90.0
DEPTH_MAX_RANGE = 20.0
C_ENGINE_MAX_SPEED = 3.0
TERMINAL_ASSIST_AGL_M = 2.40


TERRAIN_PROFILES = {
    1: {  # City
        "name": "city",
        "transit_alt": 3.0,
        "mode_v": 95.0,
        "safety": 0.95,
        "acq_gain": 0.58,
        "acq_speed_max": 1.20,
        "approach_gate_xy": 7.5,
        "term_gate_xy": 1.10,
        "final_center_gate": 0.18,
        "settle_required": 20,
        "settle_v_tol": 0.08,
        "press_vz": -0.14,
    },
    2: {  # Open / Valley
        "name": "open_valley",
        "transit_alt": 1.2,
        "mode_v": 145.0,
        "safety": 1.05,
        "acq_gain": 0.55,
        "acq_speed_max": 1.15,
        "approach_gate_xy": 4.75,
        "term_gate_xy": 1.45,
        "final_center_gate": 0.22,
        "settle_required": 10,
        "settle_v_tol": 0.10,
        "press_vz": -0.16,
    },
    3: {  # Mountain
        "name": "mountain",
        "transit_alt": 6.3,
        "mode_v": 108.0,
        "safety": 1.28,
        "acq_gain": 0.64,
        "acq_speed_max": 1.55,
        "approach_gate_xy": 22.0,
        "term_gate_xy": 1.20,
        "final_center_gate": 0.20,
        "settle_required": 12,
        "settle_v_tol": 0.10,
        "press_vz": -0.16,
        "ridge_floor_agl": 1.60,
    },
    4: {  # Village
        "name": "village",
        "transit_alt": 3.2,
        "mode_v": 100.0,
        "safety": 0.95,
        "acq_gain": 0.60,
        "acq_speed_max": 1.25,
        "approach_gate_xy": 8.0,
        "term_gate_xy": 1.10,
        "final_center_gate": 0.18,
        "settle_required": 20,
        "settle_v_tol": 0.08,
        "press_vz": -0.14,
    },
    5: {  # Warehouse
        "name": "warehouse",
        "transit_alt": 2.4,
        "mode_v": 58.0,
        "safety": 0.55,
        "acq_gain": 0.52,
        "acq_speed_max": 0.95,
        "approach_gate_xy": 4.75,
        "term_gate_xy": 0.34,
        "final_center_gate": 0.12,
        "settle_required": 24,
        "settle_v_tol": 0.055,
        "press_vz": -0.10,
    },
    6: {  # Forest
        "name": "forest",
        "transit_alt": 1.6,
        "mode_v": 125.0,
        "safety": 1.00,
        "acq_gain": 0.55,
        "acq_speed_max": 1.15,
        "approach_gate_xy": 5.5,
        "term_gate_xy": 1.25,
        "final_center_gate": 0.20,
        "settle_required": 12,
        "settle_v_tol": 0.10,
        "press_vz": -0.16,
    },
}

DEFAULT_PROFILE_ID = 2


def _profile(terrain_id: int) -> Dict[str, float]:
    return TERRAIN_PROFILES.get(int(terrain_id), TERRAIN_PROFILES[DEFAULT_PROFILE_ID])


def _safe_norm(v: np.ndarray, eps: float = 1e-8) -> float:
    return float(np.linalg.norm(v)) + eps


def _unit(v: np.ndarray) -> np.ndarray:
    return v / _safe_norm(v)


def _depth_metrics(depth: Optional[np.ndarray]) -> Tuple[float, float, float]:
    """Return close_fraction, mid_fraction, mean_depth_norm from normalized depth."""
    if depth is None:
        return 0.0, 0.0, 1.0
    d = np.asarray(depth, dtype=np.float32)
    if d.size == 0:
        return 0.0, 0.0, 1.0
    d = np.nan_to_num(d, nan=1.0, posinf=1.0, neginf=0.0)
    d = np.clip(d, 0.0, 1.0)
    close_fraction = float(np.mean(d < 0.18))
    mid_fraction = float(np.mean(d < 0.35))
    mean_depth = float(np.mean(d))
    return close_fraction, mid_fraction, mean_depth


def _action_from_velocity(vx: float, vy: float, vz: float, total_speed: float, yaw_cmd: float) -> np.ndarray:
    vel = np.array([vx, vy, vz], dtype=np.float64)
    dir_xyz = _unit(vel)
    speed_norm = float(np.clip(total_speed / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))
    return np.array([dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm], dtype=np.float32)


def _direct_action(
    current_pos: np.ndarray,
    target_pos: np.ndarray,
    target_vel: np.ndarray,
    yaw_norm: float,
    *,
    descent: bool = False,
    target_alt_override: Optional[float] = None,
    settle_ticks: int = 0,
    terrain_id: int = DEFAULT_PROFILE_ID,
) -> np.ndarray:
    """Closed-loop approach/touchdown assist outside the native potential-field controller."""
    profile = _profile(terrain_id)
    delta = target_pos - current_pos
    xy_dist = float(np.linalg.norm(delta[:2]))
    h_rem = max(float(current_pos[2] - target_pos[2]), 0.0)

    if descent:
        final_center_gate = float(profile["final_center_gate"])
        settle_required = int(profile["settle_required"])
        press_vz = float(profile["press_vz"])

        if xy_dist < final_center_gate:
            if settle_ticks >= settle_required:
                if terrain_id == 5:
                    if h_rem > 0.22:
                        vz = press_vz
                    elif h_rem > 0.08:
                        vz = -0.08
                    else:
                        vz = -0.05
                else:
                    vz = press_vz if h_rem > 0.12 else (press_vz * 0.60)
            else:
                vz = -0.003 if terrain_id == 5 else -0.01
            desired = np.array([target_vel[0], target_vel[1], vz + target_vel[2]], dtype=np.float64)
        else:
            xy_speed = min(0.22 if terrain_id == 5 else 0.55, max(0.04, xy_dist * 0.40))
            vz = -0.005 if terrain_id == 5 else (-0.10 if h_rem > 0.38 else -0.04)
            desired = np.array(
                [delta[0] / (xy_dist + 1e-8) * xy_speed + target_vel[0],
                 delta[1] / (xy_dist + 1e-8) * xy_speed + target_vel[1],
                 vz + target_vel[2]],
                dtype=np.float64,
            )
    else:
        xy_speed = min(float(profile["acq_speed_max"]), max(0.35, xy_dist * float(profile["acq_gain"])))
        desired_z = target_alt_override if target_alt_override is not None else (target_pos[2] + 1.4)

        mountain_descent_corridor = terrain_id == 3 and xy_dist < 5.0 and h_rem > 4.2
        centered_high_mountain = terrain_id == 3 and xy_dist < 2.0 and h_rem > 4.2
        descent_gain = 1.05 if centered_high_mountain else (0.75 if mountain_descent_corridor else 0.70)
        max_desc = -0.95 if centered_high_mountain else (-0.55 if mountain_descent_corridor else -0.35)

        vz = float(np.clip((desired_z - current_pos[2]) * descent_gain, max_desc, 0.55))
        desired = np.array(
            [delta[0] / (xy_dist + 1e-8) * xy_speed + target_vel[0],
             delta[1] / (xy_dist + 1e-8) * xy_speed + target_vel[1],
             vz + target_vel[2]],
            dtype=np.float64,
        )

    vel_mag = float(np.linalg.norm(desired)) + 1e-8
    dir_xyz = desired / vel_mag
    speed_norm = float(np.clip(vel_mag / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    return np.array([dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm], dtype=np.float32)


def _landing_params(terrain_id: int, dist_xy: float, spawn_z: float, target_vel: np.ndarray) -> Dict[str, object]:
    profile = _profile(terrain_id)
    landing_committed = dist_xy < 4.25

    if landing_committed:
        cruise_altitude = 0.50
        safety_radius = 0.52 if terrain_id == 5 else 0.62
        mode_v = 36.0 if terrain_id == 5 else 42.0
        attraction_gain = 16.0
    else:
        cruise_altitude = spawn_z + float(profile["transit_alt"])
        safety_radius = float(profile["safety"])
        mode_v = float(profile["mode_v"])
        attraction_gain = 13.5

    return {
        "cruise_altitude": cruise_altitude,
        "max_speed": 3.0,
        "safety_radius": safety_radius,
        "mode_v": mode_v,
        "attraction_gain": attraction_gain,
        "landing_threshold_xy": 0.95 if landing_committed else 0.75,
        "landing_threshold_z": 1.20,
        "landing_descent_rate": 0.34 if terrain_id == 5 and landing_committed else (0.38 if landing_committed else 0.34),
        "platform_vel_est": list(np.asarray(target_vel, dtype=np.float64)),
    }


class DroneFlightController:
    """Production CAE controller for Swarm Subnet model submission."""

    def __init__(self):
        self.engine = ConstrainedAdaptiveEngine()
        self.engine.start_adaptive()
        self.reset()

    def reset(self):
        if hasattr(self, "engine") and self.engine is not None:
            try:
                self.engine.reset()
                self.engine.start_adaptive()
            except Exception:
                self.engine.close()
                self.engine = ConstrainedAdaptiveEngine()
                self.engine.start_adaptive()

        self.step_count = 0
        self.spawn_z: Optional[float] = None
        self.target_est: Optional[np.ndarray] = None
        self.prev_target_est: Optional[np.ndarray] = None
        self.target_vel = np.zeros(3, dtype=np.float64)
        self.terrain_id = DEFAULT_PROFILE_ID
        self.min_dist_xy = 9999.0
        self.descent_corridor_ticks = 0
        self.terminal_ticks = 0
        self.settle_ticks = 0
        self.last_action = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    def _extract_state(self, observation: Dict[str, np.ndarray]) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float, np.ndarray]:
        state = np.asarray(observation["state"], dtype=np.float64).reshape(-1)
        current_pos = state[0:3] if state.size >= 3 else np.zeros(3, dtype=np.float64)
        current_rpy = state[3:6] if state.size >= 6 else np.zeros(3, dtype=np.float64)
        current_vel = state[6:9] if state.size >= 9 else np.zeros(3, dtype=np.float64)

        # Swarm template documents angular velocities after linear velocities.
        # Local harness historically used index 9 as yaw rate, so use index 11
        # when present and fall back safely.
        yaw_rate = float(state[11]) if state.size > 11 else (float(state[9]) if state.size > 9 else 0.0)

        # Template documents normalized altitude before the final search vector.
        # The local CAE harness used state[-4] * DEPTH_MAX_RANGE. Preserve that.
        agl = float(np.clip(state[-4], 0.0, 1.5) * DEPTH_MAX_RANGE) if state.size >= 4 else float(current_pos[2])

        # Template documents search-area relative vector as the final three terms.
        search_vec = state[-3:].astype(np.float64) if state.size >= 3 else np.zeros(3, dtype=np.float64)
        return current_pos.astype(np.float64), current_rpy.astype(np.float64), current_vel.astype(np.float64), yaw_rate, agl, search_vec

    def _estimate_target(self, current_pos: np.ndarray, search_vec: np.ndarray) -> np.ndarray:
        search_vec = np.nan_to_num(search_vec.astype(np.float64), nan=0.0, posinf=0.0, neginf=0.0)
        search_norm = float(np.linalg.norm(search_vec))

        if search_norm > 0.05:
            # Swarm provides a relative search-area vector with noisy X/Y.
            raw_target = current_pos + search_vec
        elif self.target_est is not None:
            raw_target = self.target_est.copy()
        else:
            raw_target = current_pos + np.array([0.0, 0.0, -current_pos[2]], dtype=np.float64)

        # The landing platform is the endpoint; keep the estimate near ground if
        # the search vector does not explicitly carry altitude information.
        if abs(search_vec[2]) < 0.25:
            raw_target[2] = 0.20
        else:
            raw_target[2] = max(0.10, raw_target[2])

        if self.target_est is None:
            self.target_est = raw_target.astype(np.float64)
        else:
            # Track moving/noisy targets without chasing per-frame noise.
            alpha = 0.18 if np.linalg.norm(current_pos[:2] - self.target_est[:2]) > 4.0 else 0.08
            self.target_est = (1.0 - alpha) * self.target_est + alpha * raw_target

        if self.prev_target_est is not None:
            self.target_vel = np.clip((self.target_est - self.prev_target_est) * 50.0, -1.0, 1.0)
        else:
            self.target_vel = np.zeros(3, dtype=np.float64)
        self.prev_target_est = self.target_est.copy()
        return self.target_est.copy()

    def _infer_terrain(self, current_pos: np.ndarray, agl: float, depth: Optional[np.ndarray], dist_xy: float) -> int:
        close_fraction, mid_fraction, mean_depth = _depth_metrics(depth)

        if self.spawn_z is None:
            self.spawn_z = float(current_pos[2])

        # Flight-recorder traces showed the city seed being classified as forest
        # from frame zero because close_fraction won before any city geometry cue.
        # City spawns in the large urban coordinate envelope; classify that before
        # the forest close-obstacle fallback.
        xy_extent = float(np.max(np.abs(current_pos[:2]))) if current_pos.size >= 2 else 0.0
        urban_extent = xy_extent > 18.0

        # Avoid false mountain transitions on open maps when early commands raise
        # AGL. Require high spawn altitude or mountain-like initial altitude.
        if self.spawn_z > 18.0 or (self.spawn_z > 11.5 and agl > 7.0):
            return 3  # Mountain-like high terrain case.

        if self.spawn_z < 5.0 and mid_fraction > 0.10 and dist_xy < 8.0:
            return 5  # Warehouse-like close clutter / low ceiling.

        if urban_extent and close_fraction > 0.04 and dist_xy < 14.0:
            return 1  # City: large coordinate envelope with close urban clutter.

        if mid_fraction > 0.20 and mean_depth < 0.55:
            return 1  # Dense city/village clutter; city is more conservative.

        if close_fraction > 0.18 and mid_fraction > 0.05 and mean_depth < 0.72:
            return 6  # Forest-like close obstacles; avoid open-map false positives.

        if mid_fraction > 0.08:
            return 4  # Village-like medium clutter.

        return 2

    def _yaw_command(self, current_pos: np.ndarray, target_pos: np.ndarray) -> float:
        delta = target_pos[:2] - current_pos[:2]
        if float(np.linalg.norm(delta)) < 1e-5:
            return 0.0
        yaw = math.atan2(float(delta[1]), float(delta[0]))
        return float(np.clip(yaw / math.pi, -1.0, 1.0))

    def act(self, observation: Dict[str, np.ndarray]) -> np.ndarray:
        depth = observation.get("depth")
        current_pos, current_rpy, current_vel, yaw_rate, agl, search_vec = self._extract_state(observation)

        if self.spawn_z is None:
            self.spawn_z = float(current_pos[2])

        target_pos = self._estimate_target(current_pos, search_vec)
        dist_xy = float(np.linalg.norm(current_pos[:2] - target_pos[:2]))
        self.min_dist_xy = min(self.min_dist_xy, dist_xy)

        terrain_id = self._infer_terrain(current_pos, agl, depth, dist_xy)
        self.terrain_id = terrain_id
        profile = _profile(terrain_id)

        self.engine.set_flight_params(**_landing_params(terrain_id, dist_xy, self.spawn_z, self.target_vel))
        self.engine.set_target_pos(target_pos)
        self.engine.process_sensor_data(
            current_pos,
            current_vel,
            current_rpy,
            target_pos,
            yaw_rate,
            agl,
            depth_image=depth,
            depth_width=128,
            depth_height=128,
            max_range=DEPTH_MAX_RANGE,
            fov_deg=CAMERA_FOV_DEG,
        )
        self.engine.update()

        cs = self.engine.get_state()
        if cs is None:
            return self.last_action.copy()

        vx, vy, vz, total_speed, yaw_cmd = cs.control_output
        yaw_norm = self._yaw_command(current_pos, target_pos) if abs(float(yaw_cmd)) < 1e-4 else float(np.clip(yaw_cmd, -1.0, 1.0))
        action = _action_from_velocity(vx, vy, vz, total_speed, yaw_norm)

        target_rel_v = current_vel - self.target_vel
        is_near_ground_level = current_pos[2] < (target_pos[2] + 4.2)

        mountain_low_agl_guard = (
            terrain_id == 3
            and agl < float(profile.get("ridge_floor_agl", 1.6))
            and dist_xy > 2.5
            and not bool(cs.landing_phase)
        )
        mountain_safety_climb = (
            terrain_id == 3
            and 4.5 < dist_xy < 15.0
            and (float(cs.collision_risk_score) > 0.70 or mountain_low_agl_guard)
            and current_pos[2] < target_pos[2] + 6.3
        )
        mountain_descent_corridor = (
            terrain_id == 3
            and dist_xy < 5.0
            and current_pos[2] > target_pos[2] + 4.2
            and agl > float(profile.get("ridge_floor_agl", 1.6))
        )
        mountain_centered_descent = (
            terrain_id == 3
            and dist_xy < 0.35
            and agl > TERMINAL_ASSIST_AGL_M
            and agl > float(profile.get("ridge_floor_agl", 1.6)) + 0.45
            and self.descent_corridor_ticks > 40
            and not bool(cs.landing_phase)
        )

        term_gate_xy = float(profile["term_gate_xy"])
        settle_gate = float(profile["final_center_gate"])
        settle_v_tol = float(profile["settle_v_tol"])

        warehouse_terminal_capture = (
            terrain_id == 5
            and dist_xy < term_gate_xy
            and agl < TERMINAL_ASSIST_AGL_M
            and is_near_ground_level
        )
        nonwarehouse_terminal_capture = (
            terrain_id != 5
            and dist_xy < term_gate_xy
            and current_pos[2] < target_pos[2] + 5.0
        )

        if warehouse_terminal_capture or nonwarehouse_terminal_capture:
            self.terminal_ticks += 1
            if dist_xy < settle_gate and abs(float(target_rel_v[0])) < settle_v_tol and abs(float(target_rel_v[1])) < settle_v_tol:
                self.settle_ticks += 1
            else:
                self.settle_ticks = 0

            warehouse_terminal_commit = (
                terrain_id == 5
                and self.terminal_ticks > 900
                and dist_xy < 0.50
                and agl < 0.90
                and bool(cs.landing_phase)
                and bool(cs.descent_phase)
            )
            nonwarehouse_terminal_commit = (
                terrain_id != 5
                and nonwarehouse_terminal_capture
                and dist_xy < settle_gate
                and current_pos[2] < target_pos[2] + 4.0
            )
            forced_settle = int(profile["settle_required"]) if (warehouse_terminal_commit or nonwarehouse_terminal_commit) else self.settle_ticks
            action = _direct_action(
                current_pos,
                target_pos,
                self.target_vel,
                yaw_norm,
                descent=True,
                settle_ticks=forced_settle,
                terrain_id=terrain_id,
            )
        elif (
            dist_xy < float(profile.get("approach_gate_xy", 4.75))
            or mountain_safety_climb
            or mountain_descent_corridor
            or mountain_centered_descent
        ) and not bool(cs.landing_phase):
            if mountain_low_agl_guard:
                target_alt_assist = current_pos[2] + 2.0
            elif mountain_safety_climb:
                target_alt_assist = max(current_pos[2] + 1.2, target_pos[2] + 6.3)
            elif mountain_centered_descent:
                target_alt_assist = target_pos[2] + 2.15
            elif mountain_descent_corridor:
                target_alt_assist = target_pos[2] + 4.6
            elif terrain_id == 3 and dist_xy > 5.0:
                target_alt_assist = target_pos[2] + 6.3
            else:
                target_alt_assist = target_pos[2] + 1.4

            action = _direct_action(
                current_pos,
                target_pos,
                self.target_vel,
                yaw_norm,
                descent=False,
                target_alt_override=target_alt_assist,
                terrain_id=terrain_id,
            )

        if mountain_descent_corridor:
            self.descent_corridor_ticks += 1

        self.step_count += 1
        action = np.nan_to_num(action, nan=0.0, posinf=1.0, neginf=-1.0).astype(np.float32)
        action[:3] = np.clip(action[:3], -1.0, 1.0)
        action[3] = np.clip(action[3], 0.0, 1.0)
        action[4] = np.clip(action[4], -1.0, 1.0)
        self.last_action = action.copy()
        return action
