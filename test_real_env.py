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
  downward velocity commands at spawn (which would cause excessive tilt and
  early truncation by the environment's safety cut-off).
- The bridge maps C engine output [vx, vy, vz, speed, yaw] to the
  environment's [dir_x, dir_y, dir_z, speed_norm] action format.
- No artificial speed clamps or overrides — the engine's adaptive dynamics
  manage acceleration and deceleration.
"""
import sys
import time
import numpy as np

sys.path.append('/home/ubuntu/swarm-subnet')
sys.path.append('/home/ubuntu/constrained-adaptive-engine')

from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine

# Camera constants
CAMERA_FOV_DEG  = 90.0   # Base FOV used for depth de-projection
DEPTH_MAX_RANGE = 20.0   # metres — matches swarm-subnet depth encoding

# C engine internal max speed (from adaptation_controller.c adapt_init defaults)
C_ENGINE_MAX_SPEED = 8.0


def run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=False):
    """
    Run a single trial of the CAE against the real swarm environment.

    Parameters
    ----------
    terrain_id : int   Swarm-subnet challenge type (1=City, 2=Open/Valley, …)
    seed       : int   Map seed
    max_steps  : int   Hard step cap (default 3000 = 60 s at 50 Hz)
    verbose    : bool  Print per-100-step progress

    Returns
    -------
    dict with keys: terrain_id, seed, status, steps, duration_s, min_dist_m
    """
    task = task_for_seed_and_type(sim_dt=1/50, seed=seed, challenge_type=terrain_id)
    env  = make_env(task, gui=False)
    obs, _ = env.reset()
    goal = env.GOAL_POS.copy().astype(np.float64)

    # Read spawn altitude from the first observation so the Z-attraction term
    # starts at rest (desired_vz ≈ 0) rather than commanding a dive.
    spawn_z = float(obs['state'][2])

    engine = ConstrainedAdaptiveEngine()
    engine.set_flight_params(
        cruise_altitude      = spawn_z,   # ← key fix: match spawn altitude
        max_speed            = 3.0,       # m/s — matches env SPEED_LIMIT
        safety_radius        = 1.15,      # m
        mode_v               = 200.0,     # repulsion strength
        attraction_gain      = 5.0,       # attraction gain
        landing_threshold_xy = 0.60,      # m
        landing_threshold_z  = 1.20,      # m
        landing_descent_rate = 0.025,     # m/s
        platform_vel_est     = [0.0, 0.0, 0.0],
    )
    engine.set_target_pos(goal)
    engine.start_adaptive()

    success   = False
    collision = False
    min_dist  = 9999.0

    start_time = time.time()
    for step in range(max_steps):
        state_vec   = obs['state']
        current_pos = state_vec[0:3].astype(np.float64)
        current_rpy = state_vec[3:6].astype(np.float64)
        current_vel = state_vec[6:9].astype(np.float64)
        yaw_rate    = float(state_vec[9])
        agl         = float(state_vec[112]) * DEPTH_MAX_RANGE

        # Pass raw depth image to C engine — all depth processing in C via
        # psmsl_depth_analyze_image() (16×16 subsampled, world-frame point cloud)
        engine.process_sensor_data(
            current_pos, current_vel, current_rpy, goal,
            yaw_rate, agl,
            depth_image  = obs['depth'],   # (128, 128, 1) float32 normalised [0,1]
            depth_width  = 128,
            depth_height = 128,
            max_range    = DEPTH_MAX_RANGE,
            fov_deg      = CAMERA_FOV_DEG,
        )
        engine.update()

        cs = engine.get_state()
        vx, vy, vz, total_speed, yaw_cmd = cs.control_output

        # Build unit direction vector from velocity components
        vel_mag  = np.sqrt(vx**2 + vy**2 + vz**2) + 1e-8
        dir_xyz  = np.array([vx, vy, vz]) / vel_mag

        # Map C engine speed [0, C_ENGINE_MAX_SPEED] → env speed_norm [0, 1]
        speed_norm = float(np.clip(total_speed / C_ENGINE_MAX_SPEED, 0.0, 1.0))

        action = np.array([[dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm]],
                          dtype=np.float32)
        obs, r, term, trunc, info = env.step(action)

        dist     = float(np.linalg.norm(obs['state'][0:3] - goal))
        min_dist = min(min_dist, dist)

        if verbose and step % 100 == 0:
            print(f"  step={step:4d} dist={dist:.2f} speed={total_speed:.2f} "
                  f"clutter={cs.clutter_density:.3f} "
                  f"nav={cs.navigability_score:.3f} "
                  f"risk={cs.collision_risk_score:.3f} "
                  f"landing={bool(cs.landing_phase)}")

        if info.get('collision', False):
            collision = True
            if verbose:
                print(f"  COLLISION at step={step} dist={dist:.2f}")
            break
        if info.get('success', False):
            success = True
            break
        if term or trunc:
            if verbose:
                print(f"  TERMINATED at step={step} dist={dist:.2f} "
                      f"term={term} trunc={trunc}")
            break

    env.close()
    duration = time.time() - start_time
    status   = "SUCCESS" if success else ("COLLISION" if collision else "TIMEOUT")

    return {
        'terrain_id' : terrain_id,
        'seed'       : seed,
        'status'     : status,
        'steps'      : step + 1,
        'duration_s' : round(duration, 2),
        'min_dist_m' : round(min_dist, 3),
    }


# Terrain ID → human-readable name (from swarm-subnet)
TERRAIN_NAMES = {
    1: "City Map",
    2: "Open/Valley",
    3: "Mountain",
    4: "Village",
    5: "Warehouse",
    6: "Forest",
}

if __name__ == '__main__':
    # Test suite: Open/Valley (regression guard) + City Map seeds (fix target)
    test_cases = [
        (2, 39259),   # Open/Valley — known regression guard
        (1, 29593),   # City Map — previously failing (city seed A)
        (1, 30269),   # City Map — previously failing (city seed B)
    ]

    print("=" * 60)
    print("CAE Real Environment Integration Test")
    print("  Depth: native C psmsl_depth_analyze_image (16×16 grid)")
    print("  Fix:   cruise_altitude = spawn_z (prevents tilt truncation)")
    print("=" * 60)

    for terrain_id, seed in test_cases:
        name = TERRAIN_NAMES.get(terrain_id, f"Terrain{terrain_id}")
        print(f"\n[{name} | Seed={seed}]")
        result = run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=True)
        print(f"  => {result['status']} | steps={result['steps']} | "
              f"min_dist={result['min_dist_m']}m | time={result['duration_s']}s")

    print("\n" + "=" * 60)
    print("Test complete.")
