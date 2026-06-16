"""
test_real_env.py
Authoritative integration test for the Constrained Adaptive Engine (CAE)
against the real swarm-subnet environment.

Key design principles
---------------------
- The C engine receives the raw normalised depth image and processes it
  natively via psmsl_depth_analyze_image() (16×16 subsampled, world-frame
  point cloud, spatial grid analysis). No Python-side depth-to-obstacle
  conversion — all depth processing is MCU-compliant inside the C engine.
- Transit altitude and approach aggressiveness are adapted by terrain class.
- The bridge maps C engine output [vx, vy, vz, speed, yaw] to the
  environment's [dir_x, dir_y, dir_z, speed_norm, yaw_norm] action format.
- The validation harness mirrors the tuned landing policy used by drone_agent.py.
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

# Harder maps need to clear obstacle fields before committing descent.
TERRAIN_PROFILES = {
    1: {"name": "city", "transit_alt": 3.0, "mode_v": 95.0, "safety": 0.95},
    3: {"name": "mountain", "transit_alt": 6.8, "mode_v": 120.0, "safety": 1.25},
    4: {"name": "village", "transit_alt": 3.2, "mode_v": 100.0, "safety": 0.95},
    5: {"name": "warehouse", "transit_alt": 2.4, "mode_v": 60.0, "safety": 0.55},
}


def _action_from_velocity(vx, vy, vz, total_speed, yaw_cmd):
    vel_mag = np.sqrt(vx**2 + vy**2 + vz**2) + 1e-8
    dir_xyz = np.array([vx, vy, vz], dtype=np.float64) / vel_mag
    speed_norm = float(np.clip(total_speed / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))
    return np.array([[dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm]], dtype=np.float32)


def _direct_action(current_pos, target_pos, plat_vel, yaw_norm, descent=False, target_alt_override=None, settle_ticks=0, terrain_id=None):
    """Closed-loop approach/touchdown assist outside the potential-field controller."""
    delta = target_pos - current_pos
    xy_dist = float(np.linalg.norm(delta[:2]))
    h_rem = max(float(current_pos[2] - target_pos[2]), 0.0)

    if descent:
        # Warehouse-specific final center
        final_center_gate = 0.15 if terrain_id == 5 else 0.22
        
        if xy_dist < final_center_gate:
            # Final Touchdown Settle: zero out relative XY velocity to avoid sliding off or clipping edges.
            if settle_ticks > 40:
                # Patient final press for Warehouse to ensure perfect centering
                press_vz = -0.10 if terrain_id == 5 else -0.16
                vz = press_vz if h_rem > 0.15 else (press_vz * 0.5)
            else:
                vz = -0.01 # Damping phase: allow lateral centering to finish
            desired = np.array([plat_vel[0], plat_vel[1], vz + plat_vel[2]], dtype=np.float64)
        else:
            # Centering approach: prioritize lateral alignment before final drop.
            xy_speed = min(0.50, max(0.12, xy_dist * 0.70))
            vz = -0.22 if h_rem > 0.38 else -0.12
            desired = np.array(
                [delta[0] / (xy_dist + 1e-8) * xy_speed + plat_vel[0],
                 delta[1] / (xy_dist + 1e-8) * xy_speed + plat_vel[1],
                 vz + plat_vel[2]],
                dtype=np.float64,
            )
    else:
        # Acquisition assist: pull toward the pad at a controlled altitude before C landing mode opens.
        xy_speed = min(1.15, max(0.35, xy_dist * 0.55))
        desired_z = target_alt_override if target_alt_override is not None else (target_pos[2] + 1.4)
        
        # Mountain descent boost: if we are over the pad but way too high, drop faster to beat the clock.
        descent_gain = 1.2 if (terrain_id == 3 and xy_dist < 2.0 and h_rem > 10.0) else 0.85
        max_desc = -1.5 if (terrain_id == 3 and xy_dist < 2.0) else -0.55
        
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


def _get_phase_reason(cs):
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
    profile = TERRAIN_PROFILES.get(terrain_id, {"name": "default", "transit_alt": 1.2, "mode_v": 145.0, "safety": 1.05})
    landing_committed = dist_xy < 4.25

    if landing_committed:
        cruise_altitude = 0.50
        safety_radius = 0.62
        mode_v = 42.0
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
        "landing_descent_rate": 0.42 if landing_committed else 0.38,
        "platform_vel_est": list(plat_vel),
    }


def run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=False):
    """
    Run a single trial of the CAE against the real swarm environment.
    """
    task = task_for_seed_and_type(sim_dt=1 / 50, seed=seed, challenge_type=terrain_id)
    env = make_env(task, gui=False)
    obs, _ = env.reset()
    goal = env.GOAL_POS.copy().astype(np.float64)
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
            plat_pos[2] = 0.2

            if prev_plat_pos is not None:
                plat_vel = (plat_pos - prev_plat_pos) / 0.02
            else:
                plat_vel = np.zeros(3)
            prev_plat_pos = plat_pos.copy()

            dist_xy = float(np.linalg.norm(current_pos[:2] - plat_pos[:2]))
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
            
            assist_type = 0 # 0: None, 1: Approach, 2: Terminal
            # Robust Landing Gate: AGL must be low AND absolute altitude must be near target altitude.
            # This prevents "landing" on mountain peaks 20m above the actual pad.
            is_near_ground_level = current_pos[2] < (plat_pos[2] + 4.2)
            
            # Mountain-only Safety Assist: climb/hold altitude instead of descending when collision risk is high near pad.
            mountain_safety_climb = False
            if terrain_id == 3 and dist_xy < 15.0 and cs.collision_risk_score > 0.70 and current_pos[2] > plat_pos[2] + 4.0:
                mountain_safety_climb = True

            # Warehouse-specific terminal gates
            term_gate_xy = 0.75 if terrain_id == 5 else TERMINAL_ASSIST_XY_M
            
            if dist_xy < term_gate_xy and agl < TERMINAL_ASSIST_AGL_M and is_near_ground_level:
                terminal_ticks += 1
                assist_type = 2
                
                # Settle counter logic
                rel_vx = current_vel[0] - plat_vel[0]
                rel_vy = current_vel[1] - plat_vel[1]
                if dist_xy < 0.18 and abs(rel_vx) < 0.08 and abs(rel_vy) < 0.08:
                    settle_ticks += 1
                else:
                    settle_ticks = 0
                
                action = _direct_action(current_pos, plat_pos, plat_vel, yaw_norm, descent=True, settle_ticks=settle_ticks, terrain_id=terrain_id)
            elif (dist_xy < APPROACH_ASSIST_XY_M or mountain_safety_climb) and not bool(cs.landing_phase):
                approach_assist_ticks += 1
                assist_type = 1
                # If mountain safety climb is active, force a higher target altitude in the assist.
                target_alt_assist = plat_pos[2] + (5.5 if mountain_safety_climb else 1.4)
                action = _direct_action(current_pos, plat_pos, plat_vel, yaw_norm, descent=False, target_alt_override=target_alt_assist)

            dist = float(np.linalg.norm(current_pos - plat_pos))
            phase_reason = _get_phase_reason(cs)
            rel_vx_to_pad = current_vel[0] - plat_vel[0]
            rel_vy_to_pad = current_vel[1] - plat_vel[1]
            recorder.record_step(step, time.time() - start_time, current_pos, current_vel, 
                               agl, dist_xy, dist, plat_pos, plat_vel, cs.control_output, cs, assist_type, 
                               terrain_id, phase_reason, rel_vx_to_pad, rel_vy_to_pad)

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
        (3, 1337),   # Mountain Collision
        (5, 1337),   # Warehouse Collision
        (2, 39259),  # Open/Valley Collision (agl clipping case)
    ]

    print("=" * 60)
    print("CAE Real Environment Integration Test")
    print("  Depth: native C psmsl_depth_analyze_image (16×16 grid)")
    print("  Landing: terrain-aware approach + soft terminal assist")
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
            f"time={result['duration_s']}s"
        )

    print("\n" + "=" * 60)
    print("Test complete.")
