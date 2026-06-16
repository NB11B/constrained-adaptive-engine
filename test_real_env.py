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
- cruise_altitude is initialised from the spawn altitude observed in the
  first observation. This prevents the Z-attraction term from issuing large
  downward velocity commands at spawn.
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

CAMERA_FOV_DEG = 90.0
DEPTH_MAX_RANGE = 20.0
C_ENGINE_MAX_SPEED = 3.0
TERMINAL_ASSIST_XY_M = 1.25
TERMINAL_ASSIST_AGL_M = 2.20


def _action_from_velocity(vx, vy, vz, total_speed, yaw_cmd):
    vel_mag = np.sqrt(vx**2 + vy**2 + vz**2) + 1e-8
    dir_xyz = np.array([vx, vy, vz], dtype=np.float64) / vel_mag
    speed_norm = float(np.clip(total_speed / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    yaw_norm = float(np.clip(yaw_cmd, -1.0, 1.0))
    return np.array([[dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm]], dtype=np.float32)


def _terminal_assist_action(current_pos, target_pos, yaw_norm):
    delta = target_pos - current_pos
    xy_dist = float(np.linalg.norm(delta[:2]))
    h_rem = max(float(current_pos[2] - target_pos[2]), 0.0)

    if xy_dist < 0.18:
        desired = np.array([0.0, 0.0, -0.55 if h_rem > 0.35 else -0.28], dtype=np.float64)
    else:
        xy_speed = min(0.65, max(0.12, xy_dist * 0.85))
        desired = np.array(
            [delta[0] / (xy_dist + 1e-8) * xy_speed,
             delta[1] / (xy_dist + 1e-8) * xy_speed,
             -0.55 if h_rem > 0.35 else -0.25],
            dtype=np.float64,
        )

    vel_mag = float(np.linalg.norm(desired)) + 1e-8
    dir_xyz = desired / vel_mag
    speed_norm = float(np.clip(vel_mag / C_ENGINE_MAX_SPEED, 0.0, 1.0))
    return np.array([[dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_norm]], dtype=np.float32)


def _landing_params(dist_xy, spawn_z, plat_vel):
    landing_committed = dist_xy < 3.0
    return {
        "cruise_altitude": 0.55 if landing_committed else spawn_z,
        "max_speed": 3.0,
        "safety_radius": 0.70 if landing_committed else 1.10,
        "mode_v": 55.0 if landing_committed else 160.0,
        "attraction_gain": 15.0 if landing_committed else 11.0,
        "landing_threshold_xy": 0.85 if landing_committed else 0.65,
        "landing_threshold_z": 1.20,
        "landing_descent_rate": 0.58 if landing_committed else 0.45,
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
    engine.set_flight_params(**_landing_params(dist_xy=999.0, spawn_z=spawn_z, plat_vel=np.zeros(3)))
    engine.set_target_pos(goal)
    engine.start_adaptive()

    success = False
    collision = False
    min_dist = 9999.0
    prev_plat_pos = None
    last_landing = False
    last_descent = False

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
            engine.set_flight_params(**_landing_params(dist_xy=dist_xy, spawn_z=spawn_z, plat_vel=plat_vel))
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

            if dist_xy < TERMINAL_ASSIST_XY_M and agl < TERMINAL_ASSIST_AGL_M:
                action = _terminal_assist_action(current_pos, plat_pos, yaw_norm)

            obs, _reward, term, trunc, info = env.step(action)

            dist = float(np.linalg.norm(obs["state"][0:3] - plat_pos))
            min_dist = min(min_dist, dist)
            last_landing = bool(cs.landing_phase)
            last_descent = bool(cs.descent_phase)

            if verbose and (step % 25 == 0 or last_landing or last_descent):
                print(
                    f"  step={step:4d} pos={current_pos.round(3)} "
                    f"dist_xy={dist_xy:.2f} agl={agl:.2f} "
                    f"vel={current_vel.round(3)} action={action.round(3)} "
                    f"landing={last_landing} descent={last_descent} "
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
        (2, 39259),
        (1, 29593),
        (1, 30269),
    ]

    print("=" * 60)
    print("CAE Real Environment Integration Test")
    print("  Depth: native C psmsl_depth_analyze_image (16×16 grid)")
    print("  Landing: tuned committed descent + terminal assist")
    print("=" * 60)

    for terrain_id, seed in test_cases:
        name = TERRAIN_NAMES.get(terrain_id, f"Terrain{terrain_id}")
        print(f"\n[{name} | Seed={seed}]")
        result = run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=True)
        print(
            f"  => {result['status']} | steps={result['steps']} | "
            f"min_dist={result['min_dist_m']}m | "
            f"landing={result['landing_phase']} descent={result['descent_phase']} | "
            f"time={result['duration_s']}s"
        )

    print("\n" + "=" * 60)
    print("Test complete.")
