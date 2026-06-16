"""
test_real_env.py
Authoritative integration test for the Constrained Adaptive Engine (CAE)
against the real swarm-subnet environment.

This harness is intentionally explicit about terrain-specific policy because the
adaptive engine is being validated against very different obstacle geometries:
open terrain, dense urban/village clutter, steep mountain ridges, and warehouse
pad-edge constraints.
"""
import os
import sys
import time

import numpy as np

_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(_dir)

# Optional portable path hook for local source checkouts:
#   SWARM_REPO=/path/to/swarm python3 test_real_env.py
_swarm_repo = os.environ.get("SWARM_REPO")
if _swarm_repo:
    sys.path.insert(0, os.path.abspath(_swarm_repo))

from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine
from flight_recorder import FlightRecorder

CAMERA_FOV_DEG = 90.0
DEPTH_MAX_RANGE = 20.0
C_ENGINE_MAX_SPEED = 3.0
APPROACH_ASSIST_XY_M = 4.75
TERMINAL_ASSIST_XY_M = 1.45
TERMINAL_ASSIST_AGL_M = 2.40

# Stable terrain profiles. Mountain keeps a terrain-clearance floor before the
# landing corridor. Warehouse uses a centered, softer touchdown that is decisive
# enough to avoid hover timeouts without edge-clipping the pad.
TERRAIN_PROFILES = {
    1: {
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
    2: {
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
    3: {
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
    4: {
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
    5: {
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
    6: {
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

DEFAULT_PROFILE = TERRAIN_PROFILES[2]


def _profile(terrain_id):
    return TERRAIN_PROFILES.get(terrain_id, DEFAULT_PROFILE)


def _action_from_velocity(vx, vy, vz, total_speed, yaw_cmd):
    vel_mag = np.sqrt(vx**2 + vy**2 + vz**2) + 1e-8
    dir_xyz = np.array([vx, vy, vz], dtype=np.float64) / vel_mag
    speed_norm = float(np.clip(total_speed / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))
    return np.array([[dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm]], dtype=np.float32)


def _direct_action(
    current_pos,
    target_pos,
    plat_vel,
    yaw_norm,
    *,
    descent=False,
    target_alt_override=None,
    settle_ticks=0,
    terrain_id=None,
):
    """Closed-loop approach/touchdown assist outside the potential-field controller."""
    profile = _profile(terrain_id)
    delta = target_pos - current_pos
    xy_dist = float(np.linalg.norm(delta[:2]))
    h_rem = max(float(current_pos[2] - target_pos[2]), 0.0)

    if descent:
        final_center_gate = profile["final_center_gate"]
        settle_required = profile["settle_required"]
        press_vz = profile["press_vz"]

        if xy_dist < final_center_gate:
            # Centered enough: first damp/settle relative XY, then press vertically.
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
            desired = np.array([plat_vel[0], plat_vel[1], vz + plat_vel[2]], dtype=np.float64)
        else:
            # Still outside final center gate. Prioritize centering over dropping.
            xy_speed = min(0.22 if terrain_id == 5 else 0.55, max(0.04, xy_dist * 0.40))
            vz = -0.005 if terrain_id == 5 else (-0.10 if h_rem > 0.38 else -0.04)
            desired = np.array(
                [delta[0] / (xy_dist + 1e-8) * xy_speed + plat_vel[0],
                 delta[1] / (xy_dist + 1e-8) * xy_speed + plat_vel[1],
                 vz + plat_vel[2]],
                dtype=np.float64,
            )
    else:
        # Acquisition assist: pull toward the pad while holding a controlled altitude.
        xy_speed = min(profile["acq_speed_max"], max(0.35, xy_dist * profile["acq_gain"]))
        desired_z = target_alt_override if target_alt_override is not None else (target_pos[2] + 1.4)

        # Mountain-specific descent corridor: only descend meaningfully once within
        # the approach cylinder, and never use this path to fly terrain-hugging.
        mountain_descent_corridor = terrain_id == 3 and xy_dist < 5.0 and h_rem > 4.2
        centered_high_mountain = terrain_id == 3 and xy_dist < 2.0 and h_rem > 4.2
        descent_gain = 1.05 if centered_high_mountain else (0.75 if mountain_descent_corridor else 0.70)
        max_desc = -0.95 if centered_high_mountain else (-0.55 if mountain_descent_corridor else -0.35)

        vz = float(np.clip((desired_z - current_pos[2]) * descent_gain, max_desc, 0.55))
        desired = np.array(
            [delta[0] / (xy_dist + 1e-8) * xy_speed + plat_vel[0],
             delta[1] / (xy_dist + 1e-8) * xy_speed + plat_vel[1],
             vz + plat_vel[2]],
            dtype=np.float64,
        )

    vel_mag = float(np.linalg.norm(desired)) + 1e-8
    dir_xyz = desired / vel_mag
    speed_norm = float(np.clip(vel_mag / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    return np.array([[dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm]], dtype=np.float32)


def _c_phase_reason(cs):
    if cs.target_reached:
        return "TARGET_REACHED"
    if cs.descent_phase:
        return "DESCENT_PHASE"
    if cs.landing_phase:
        return "LANDING_PHASE"
    if cs.escape_mode_active:
        return "ESCAPE_ACTIVE"
    if cs.predictive_escape_active:
        return "PREDICTIVE_ESCAPE_ACTIVE"
    if cs.collision_detected:
        return "COLLISION_DETECTED"
    return "SEARCH_PHASE"


def _landing_params(terrain_id, dist_xy, spawn_z, plat_vel):
    profile = _profile(terrain_id)
    landing_committed = dist_xy < 4.25

    if landing_committed:
        cruise_altitude = 0.50
        safety_radius = 0.52 if terrain_id == 5 else 0.62
        mode_v = 36.0 if terrain_id == 5 else 42.0
        attraction_gain = 16.0
    else:
        cruise_altitude = spawn_z + profile["transit_alt"]
        safety_radius = profile["safety"]
        mode_v = profile["mode_v"]
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
        "platform_vel_est": list(plat_vel),
    }


def run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=False):
    """Run a single trial of the CAE against the real swarm environment."""
    task = task_for_seed_and_type(sim_dt=1 / 50, seed=seed, challenge_type=terrain_id)
    env = make_env(task, gui=False)
    obs, _ = env.reset()
    goal = env.GOAL_POS.copy().astype(np.float64)
    if terrain_id != 3:
        goal[2] = 0.2

    spawn_z = float(obs["state"][2])

    engine = ConstrainedAdaptiveEngine()
    engine.set_flight_params(**_landing_params(terrain_id, dist_xy=999.0, spawn_z=spawn_z, plat_vel=np.zeros(3)))
    engine.set_target_pos(goal)
    engine.start_adaptive()

    success = False
    collision = False
    min_dist = 9999.0
    prev_plat_pos = None
    last_landing = False
    last_descent = False
    terminal_ticks = 0
    approach_assist_ticks = 0
    settle_ticks = 0
    last_phase_reason = "INIT"
    min_dist_step = 0
    min_dist_xy = 9999.0
    min_agl = 9999.0
    agl_floor_ticks = 0
    descent_corridor_ticks = 0
    terminal_press_ticks = 0
    settle_phase_ticks = 0
    warehouse_commit_ticks = 0

    recorder = FlightRecorder()
    terrain_name = TERRAIN_NAMES.get(terrain_id, f"Terrain{terrain_id}")
    recorder.start_trial(terrain_name.replace(" ", "_"), seed)

    start_time = time.time()
    try:
        for step in range(max_steps):
            state_vec = obs["state"]
            current_pos = state_vec[0:3].astype(np.float64)
            current_rpy = state_vec[3:6].astype(np.float64)
            current_vel = state_vec[6:9].astype(np.float64)
            yaw_rate = float(state_vec[9])
            agl = float(state_vec[-4]) * DEPTH_MAX_RANGE

            import pybullet as p

            plat_uid = env.unwrapped._end_platform_uids[-1]
            plat_pos_raw, _ = p.getBasePositionAndOrientation(
                plat_uid,
                physicsClientId=env.unwrapped.CLIENT,
            )
            plat_pos = np.array(plat_pos_raw, dtype=np.float64)
            if terrain_id != 3:
                plat_pos[2] = 0.2

            if prev_plat_pos is not None:
                plat_vel = (plat_pos - prev_plat_pos) / 0.02
            else:
                plat_vel = np.zeros(3)
            prev_plat_pos = plat_pos.copy()

            profile = _profile(terrain_id)
            dist_xy = float(np.linalg.norm(current_pos[:2] - plat_pos[:2]))
            rel_vx_to_pad = current_vel[0] - plat_vel[0]
            rel_vy_to_pad = current_vel[1] - plat_vel[1]
            pad_frame_dx = current_pos[0] - plat_pos[0]
            pad_frame_dy = current_pos[1] - plat_pos[1]

            engine.set_flight_params(**_landing_params(terrain_id, dist_xy=dist_xy, spawn_z=spawn_z, plat_vel=plat_vel))
            engine.set_target_pos(plat_pos)

            engine.process_sensor_data(
                current_pos,
                current_vel,
                current_rpy,
                plat_pos,
                yaw_rate,
                agl,
                depth_image=obs["depth"],
                depth_width=128,
                depth_height=128,
                max_range=DEPTH_MAX_RANGE,
                fov_deg=CAMERA_FOV_DEG,
            )
            engine.update()

            cs = engine.get_state()
            vx, vy, vz, total_speed, yaw_cmd = cs.control_output
            yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))
            action = _action_from_velocity(vx, vy, vz, total_speed, yaw_norm)

            assist_type = 0  # 0: none, 1: approach, 2: terminal
            phase_reason = _c_phase_reason(cs)
            is_near_ground_level = current_pos[2] < (plat_pos[2] + 4.2)

            min_agl = min(min_agl, agl)

            if dist_xy < min_dist_xy:
                min_dist_step = step
                min_dist_xy = dist_xy

            mountain_low_agl_guard = (
                terrain_id == 3
                and agl < profile.get("ridge_floor_agl", 1.6)
                and dist_xy > 2.5
                and not bool(cs.landing_phase)
            )
            mountain_safety_climb = (
                terrain_id == 3
                and 4.5 < dist_xy < 15.0
                and (cs.collision_risk_score > 0.70 or mountain_low_agl_guard)
                and current_pos[2] < plat_pos[2] + 6.3
            )

            mountain_descent_corridor = (
                terrain_id == 3
                and dist_xy < 5.0
                and current_pos[2] > plat_pos[2] + 4.2
                and agl > profile.get("ridge_floor_agl", 1.6)
            )
            approach_gate_xy = profile.get("approach_gate_xy", APPROACH_ASSIST_XY_M)
            term_gate_xy = profile["term_gate_xy"]
            settle_gate = profile["final_center_gate"]
            settle_v_tol = profile["settle_v_tol"]

            if dist_xy < term_gate_xy and agl < TERMINAL_ASSIST_AGL_M and is_near_ground_level:
                terminal_ticks += 1
                assist_type = 2

                if dist_xy < settle_gate and abs(rel_vx_to_pad) < settle_v_tol and abs(rel_vy_to_pad) < settle_v_tol:
                    settle_ticks += 1
                else:
                    settle_ticks = 0

                warehouse_terminal_commit = (
                    terrain_id == 5
                    and terminal_ticks > 900
                    and dist_xy < 0.50
                    and agl < 0.90
                    and bool(cs.landing_phase)
                    and bool(cs.descent_phase)
                )

                forced_settle_ticks = profile["settle_required"] if warehouse_terminal_commit else settle_ticks

                action = _direct_action(
                    current_pos,
                    plat_pos,
                    plat_vel,
                    yaw_norm,
                    descent=True,
                    settle_ticks=forced_settle_ticks,
                    terrain_id=terrain_id,
                )

                if warehouse_terminal_commit:
                    phase_reason = "WAREHOUSE_COMMIT_PRESS"
                    warehouse_commit_ticks += 1
                else:
                    phase_reason = "TERMINAL_PRESS" if settle_ticks >= profile["settle_required"] else "SETTLE_PHASE"
            elif (dist_xy < approach_gate_xy or mountain_safety_climb or mountain_descent_corridor) and not bool(cs.landing_phase):
                approach_assist_ticks += 1
                assist_type = 1
                if mountain_low_agl_guard:
                    target_alt_assist = current_pos[2] + 2.0
                    phase_reason = "MOUNTAIN_AGL_FLOOR_CLIMB"
                elif mountain_safety_climb:
                    target_alt_assist = max(current_pos[2] + 1.2, plat_pos[2] + 6.3)
                    phase_reason = "MOUNTAIN_SAFETY_CLIMB"
                elif mountain_descent_corridor:
                    target_alt_assist = plat_pos[2] + 4.6
                    phase_reason = "MOUNTAIN_DESCENT_CORRIDOR"
                elif terrain_id == 3 and dist_xy > 5.0:
                    target_alt_assist = plat_pos[2] + 6.3
                    phase_reason = "MOUNTAIN_APPROACH_CORRIDOR"
                else:
                    target_alt_assist = plat_pos[2] + 1.4
                    phase_reason = "ACQUISITION_ASSIST"

                action = _direct_action(
                    current_pos,
                    plat_pos,
                    plat_vel,
                    yaw_norm,
                    descent=False,
                    target_alt_override=target_alt_assist,
                    terrain_id=terrain_id,
                )

            last_phase_reason = phase_reason
            if phase_reason == "MOUNTAIN_AGL_FLOOR_CLIMB":
                agl_floor_ticks += 1
            if phase_reason == "MOUNTAIN_DESCENT_CORRIDOR":
                descent_corridor_ticks += 1
            if phase_reason == "TERMINAL_PRESS":
                terminal_press_ticks += 1
            if phase_reason == "SETTLE_PHASE":
                settle_phase_ticks += 1

            dist = float(np.linalg.norm(current_pos - plat_pos))
            recorder.record_step(
                step,
                time.time() - start_time,
                current_pos,
                current_vel,
                agl,
                dist_xy,
                dist,
                plat_pos,
                plat_vel,
                cs.control_output,
                cs,
                assist_type,
                terrain_id,
                phase_reason,
                rel_vx_to_pad,
                rel_vy_to_pad,
                pad_frame_dx,
                pad_frame_dy,
            )

            obs, _reward, term, trunc, info = env.step(action)

            dist = float(np.linalg.norm(obs["state"][0:3] - plat_pos))
            min_dist = min(min_dist, dist)
            last_landing = bool(cs.landing_phase)
            last_descent = bool(cs.descent_phase)

            if verbose and (step % 25 == 0 or last_landing or last_descent or terminal_ticks > 0):
                print(
                    f"  step={step:4d} pos={current_pos.round(3)} "
                    f"dist_xy={dist_xy:.2f} agl={agl:.2f} "
                    f"vel={current_vel.round(3)} action={action.round(3)} "
                    f"landing={last_landing} descent={last_descent} "
                    f"assist={approach_assist_ticks}/{terminal_ticks} "
                    f"phase={phase_reason} settle={settle_ticks} "
                    f"risk={cs.collision_risk_score:.2f} clutter={cs.clutter_density:.2f}"
                )

            if info.get("collision", False):
                collision = True
                if verbose:
                    print(f"  COLLISION at step={step} dist={dist:.2f}")
                break
            if info.get("success", False):
                success = True
                break
            if term or trunc:
                if verbose:
                    print(f"  TERMINATED at step={step} dist={dist:.2f} term={term} trunc={trunc}")
                break
        else:
            step = max_steps - 1
    finally:
        recorder.close()
        if hasattr(engine, "close"):
            engine.close()
        env.close()

    duration = time.time() - start_time
    status = "SUCCESS" if success else ("COLLISION" if collision else "TIMEOUT")

    return {
        "terrain_id": terrain_id,
        "seed": seed,
        "status": status,
        "steps": step + 1,
        "duration_s": round(duration, 2),
        "min_dist_m": round(min_dist, 3),
        "landing_phase": last_landing,
        "descent_phase": last_descent,
        "approach_assist_ticks": approach_assist_ticks,
        "terminal_assist_ticks": terminal_ticks,
        "wall_time_s": 0.0,
        "last_phase_reason": last_phase_reason,
        "min_dist_step": min_dist_step,
        "min_dist_xy": round(min_dist_xy, 3),
        "min_agl": round(min_agl, 3),
        "agl_floor_ticks": agl_floor_ticks,
        "descent_corridor_ticks": descent_corridor_ticks,
        "settle_phase_ticks": settle_phase_ticks,
        "terminal_press_ticks": terminal_press_ticks,
        "warehouse_commit_ticks": warehouse_commit_ticks,
    }


TERRAIN_NAMES = {
    1: "City Map",
    2: "Open/Valley",
    3: "Mountain",
    4: "Village",
    5: "Warehouse",
    6: "Forest",
}


if __name__ == "__main__":
    test_cases = [
        (3, 1337),   # Mountain regression
        (5, 1337),   # Warehouse regression
        (2, 39259),  # Open/Valley known-good guard
    ]

    print("=" * 60)
    print("CAE Real Environment Integration Test")
    print("  Depth: native C psmsl_depth_analyze_image (16×16 grid)")
    print("  Landing: mountain corridor optimization + warehouse commit guard")
    print("=" * 60)

    for terrain_id, seed in test_cases:
        name = TERRAIN_NAMES.get(terrain_id, f"Terrain{terrain_id}")
        print(f"\n[{name} | Seed={seed}]")
        result = run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=True)
        print(
            f"  => {result['status']} | steps={result['steps']} | "
            f"min_dist={result['min_dist_m']}m | "
            f"landing={result['landing_phase']} descent={result['descent_phase']} | "
            f"assist={result['approach_assist_ticks']}/{result['terminal_assist_ticks']} | "
            f"phase={result.get('last_phase_reason', 'N/A')} | "
            f"time={result['duration_s']}s"
        )

    print("\n" + "=" * 60)
    print("Test complete.")
