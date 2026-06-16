"""
benchmark_faithful.py
=====================
Faithful local benchmark for the Constrained Adaptive Engine (CAE).

This harness mirrors the Swarm subnet validator pipeline **exactly** as
implemented in:
  swarm/validator/docker/docker_evaluator_parts/rpc.py
  swarm/validator/reward.py
  swarm/validator/task_gen.py
  swarm/constants.py

Exact contracts replicated:
  - Action format:  np.array([dir_x, dir_y, dir_z, speed_norm, yaw_rate], float32)
                    clipped to env.action_space.low/high, passed as act[None, :]
  - Speed norm:     speed_norm = v_total / SPEED_LIMIT (3.0 m/s)
  - Loop condition: while t_sim < task.horizon (60 s)
  - Success:        info.get("success", False)
  - Collision:      info.get("collision", False)
  - Min clearance:  info.get("min_clearance", None)
  - Score:          flight_reward(success, t, horizon, task, min_clearance, collision)
                    = 0.45*success + 0.45*time_term + 0.10*safety_term
  - Challenge types: 1=City, 2=Open/Valley, 3=Mountain, 4=Village, 5=Warehouse, 6=Forest
  - Moving platform: type2=80%, types1/3/4=25%, types5/6=0%
  - Screening template: 50-slot repeating, no moving platforms
  - Seed manager:   seeds drawn from epoch seed list (we use fixed reproducible seeds)

Usage:
    python3 benchmark_faithful.py [--mode screening|full|custom] [--seeds N] [--types 1,2,3]

Copyright: Nathanael J. Bocker, 2026 — all rights reserved
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── Path setup ────────────────────────────────────────────────────────────────
sys.path.insert(0, '/home/ubuntu/swarm-subnet')
sys.path.insert(0, '/home/ubuntu/constrained-adaptive-engine')

# ── Swarm imports (exact same as rpc.py) ─────────────────────────────────────
from gym_pybullet_drones.utils.enums import ActionType
from swarm.constants import (
    CHALLENGE_TYPE_DISTRIBUTION,
    BENCHMARK_SCREENING_SEED_COUNT,
    BENCHMARK_TOTAL_SEED_COUNT,
    BENCHMARK_VERSION,
    HORIZON_SEC,
    HOVER_SEC,
    MOVING_PLATFORM_PROB,
    REWARD_W_SAFETY,
    REWARD_W_SUCCESS,
    REWARD_W_TIME,
    SAFETY_DISTANCE_SAFE,
    SAFETY_DISTANCE_SAFE_BY_TYPE,
    SCREENING_TEMPLATE,
    SIM_DT,
    SPEED_LIMIT,
)
from swarm.utils.env_factory import make_env_with_initial_obs
from swarm.validator.reward import flight_reward
from swarm.validator.task_gen import random_task, screening_task, task_for_seed_and_type

# ── CAE import ────────────────────────────────────────────────────────────────
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine

# ── Benchmark configuration ───────────────────────────────────────────────────
CHALLENGE_TYPE_NAMES = {
    1: "City",
    2: "Open/Valley",
    3: "Mountain",
    4: "Village",
    5: "Warehouse",
    6: "Forest",
}

# Screening template: 50-slot repeating, matches validator exactly
SCREENING_TEMPLATE_LEN = len(SCREENING_TEMPLATE)

# Camera/depth constants (match test_real_env.py)
CAMERA_FOV_DEG  = 90.0
DEPTH_MAX_RANGE = 20.0

# Output directory
RESULTS_DIR = Path(__file__).parent / "benchmark_results"
RESULTS_DIR.mkdir(exist_ok=True)


# =============================================================================
# Core trial runner — mirrors rpc.py run loop exactly
# =============================================================================

def run_trial(
    task,
    max_steps: Optional[int] = None,
    verbose: bool = False,
) -> Dict:
    """
    Run one trial of the CAE against the real Swarm environment.

    Replicates the rpc.py inner loop:
      - action clipped to env.action_space.low/high
      - speed normalised to SPEED_LIMIT
      - loop condition: t_sim < task.horizon
      - success/collision/min_clearance from info dict
      - score from flight_reward()

    Returns a dict with all fields needed for the CSV report.
    """
    env, obs_init = make_env_with_initial_obs(task, gui=False)
    obs = obs_init

    # Action space bounds (exact same as rpc.py)
    lo = env.action_space.low.flatten()
    hi = env.action_space.high.flatten()

    # Read spawn altitude for Z-attraction initialisation
    spawn_z = float(obs['state'][2])
    goal = np.array(env.GOAL_POS, dtype=np.float64)

    # Initialise CAE
    engine = ConstrainedAdaptiveEngine()
    engine.set_flight_params(
        cruise_altitude      = spawn_z,
        max_speed            = SPEED_LIMIT,
        safety_radius        = 1.15,
        mode_v               = 200.0,
        attraction_gain      = 5.0,
        landing_threshold_xy = 0.60,
        landing_threshold_z  = 1.20,
        landing_descent_rate = 0.025,
        platform_vel_est     = [0.0, 0.0, 0.0],
    )
    engine.set_target_pos(goal.tolist())
    engine.start_adaptive()

    t_sim    = 0.0
    success  = False
    collision = False
    info     = {}
    step_idx = 0
    min_dist_seen = float('inf')

    trial_start = time.time()

    # ── Main loop (exact condition from rpc.py) ───────────────────────────────
    while t_sim < task.horizon:
        step_idx += 1

        # Build observation for CAE
        state_vec  = obs['state']
        depth_img  = obs.get('depth', None)

        pos = state_vec[0:3].tolist()
        vel = state_vec[3:6].tolist()
        rpy = state_vec[6:9].tolist()
        yaw_rate = float(state_vec[9]) if len(state_vec) > 9 else 0.0
        agl = float(pos[2])

        # Feed sensor data into CAE
        if depth_img is not None:
            depth_flat = depth_img.flatten().astype(np.float32)
            h, w = depth_img.shape[:2] if depth_img.ndim >= 2 else (1, len(depth_flat))
            engine.process_sensor_data(
                current_pos  = pos,
                current_vel  = vel,
                current_rpy  = rpy,
                target_pos   = goal.tolist(),
                yaw_rate     = yaw_rate,
                agl          = agl,
                depth_image  = depth_flat,
                depth_width  = w,
                depth_height = h,
                max_range    = DEPTH_MAX_RANGE,
                fov_deg      = CAMERA_FOV_DEG,
            )
        else:
            engine.process_sensor_data(
                current_pos  = pos,
                current_vel  = vel,
                current_rpy  = rpy,
                target_pos   = goal.tolist(),
                yaw_rate     = yaw_rate,
                agl          = agl,
                depth_image  = np.zeros(16*16, dtype=np.float32),
                depth_width  = 16,
                depth_height = 16,
                max_range    = DEPTH_MAX_RANGE,
                fov_deg      = CAMERA_FOV_DEG,
            )

        engine.update()
        state = engine.get_state()
        vx, vy, vz, total_speed, yaw_cmd = state.control_output

        # ── Action construction (exact rpc.py format) ─────────────────────────
        # dir_xyz is the unit direction vector; speed_norm = v_total / SPEED_LIMIT
        if total_speed > 1e-6:
            dir_xyz = [vx / total_speed, vy / total_speed, vz / total_speed]
        else:
            dir_xyz = [1.0, 0.0, 0.0]
        speed_norm = float(np.clip(total_speed / SPEED_LIMIT, 0.0, 1.0))

        raw_act = np.array(
            [dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_cmd],
            dtype=np.float32,
        )
        # nan/inf guard (mirrors rpc.py np.nan_to_num)
        raw_act = np.nan_to_num(raw_act, nan=0.0, posinf=0.0, neginf=0.0)
        if raw_act.size != 5:
            raw_act = np.zeros(5, dtype=np.float32)

        # Clip to env action space bounds (exact rpc.py)
        act = np.clip(raw_act, lo, hi)

        # VEL ActionType speed normalisation (exact rpc.py block)
        if hasattr(env, 'ACT_TYPE') and hasattr(env, 'SPEED_LIMIT'):
            if env.ACT_TYPE == ActionType.VEL and env.SPEED_LIMIT:
                n = max(np.linalg.norm(act[:3]), 1e-6)
                scale = min(1.0, SPEED_LIMIT / n)
                act[:3] *= scale
                act = np.clip(act, lo, hi)

        # Step the environment (exact rpc.py: act[None, :])
        obs, _r, terminated, truncated, info = env.step(act[None, :])
        t_sim += SIM_DT

        # Track closest approach for safety term
        dist = float(np.linalg.norm(np.array(pos) - goal))
        if dist < min_dist_seen:
            min_dist_seen = dist

        if verbose and step_idx % 100 == 0:
            s = engine.get_state()
            print(
                f"  step={step_idx:4d} t={t_sim:.1f}s dist={dist:.3f}m "
                f"z={pos[2]:.3f} spd={total_speed:.2f} "
                f"L={int(s.landing_phase)} D={int(s.descent_phase)} "
                f"PE={int(s.predictive_escape_active)} "
                f"threat={s.threat_accumulator:.1f}"
            )

        if terminated or truncated:
            success = bool(info.get('success', False))
            break

    # ── Score (exact rpc.py flight_reward call) ───────────────────────────────
    min_clearance = info.get('min_clearance', None)
    collision     = bool(info.get('collision', False))
    score = flight_reward(
        success       = success,
        t             = t_sim,
        horizon       = task.horizon,
        task          = task,
        min_clearance = min_clearance,
        collision     = collision,
        legitimate_model = True,
    )

    wall_time = time.time() - trial_start

    return {
        'challenge_type':    int(task.challenge_type),
        'terrain_name':      CHALLENGE_TYPE_NAMES.get(task.challenge_type, '?'),
        'seed':              int(task.map_seed),
        'moving_platform':   bool(task.moving_platform),
        'horizon_s':         float(task.horizon),
        'success':           success,
        'collision':         collision,
        'steps':             step_idx,
        't_sim_s':           round(t_sim, 3),
        'wall_time_s':       round(wall_time, 2),
        'min_dist_m':        round(min_dist_seen, 4),
        'min_clearance_m':   round(min_clearance, 4) if min_clearance is not None else None,
        'score':             round(score, 6),
    }


# =============================================================================
# Benchmark modes
# =============================================================================

def build_screening_trials(seeds: List[int]) -> List[Tuple]:
    """
    Build (task, label) pairs using the SCREENING_TEMPLATE, exactly as the
    validator does for the first BENCHMARK_SCREENING_SEED_COUNT seeds.
    """
    template = (
        SCREENING_TEMPLATE * ((len(seeds) // SCREENING_TEMPLATE_LEN) + 1)
    )[:len(seeds)]
    trials = []
    for i, seed in enumerate(seeds):
        slot = template[i]
        task = screening_task(
            sim_dt           = SIM_DT,
            seed             = seed,
            challenge_type   = slot['challenge_type'],
            distance_range   = slot['distance_range'],
            goal_height_range= slot.get('goal_height_range'),
            moving_platform  = slot['moving_platform'],
        )
        trials.append((task, f"screening[{i}] type={slot['challenge_type']} seed={seed}"))
    return trials


def build_random_trials(seeds: List[int]) -> List[Tuple]:
    """
    Build (task, label) pairs using random_task(), exactly as the validator
    does for seeds beyond the screening range.
    """
    trials = []
    for seed in seeds:
        task = random_task(sim_dt=SIM_DT, seed=seed)
        trials.append((task, f"random type={task.challenge_type} seed={seed}"))
    return trials


def build_typed_trials(seeds: List[int], challenge_types: List[int]) -> List[Tuple]:
    """
    Build (task, label) pairs for explicit challenge types — used for
    per-type analysis. Moving platform probability matches validator constants.
    """
    trials = []
    for ctype in challenge_types:
        for seed in seeds:
            task = task_for_seed_and_type(
                sim_dt         = SIM_DT,
                seed           = seed,
                challenge_type = ctype,
                moving_platform= None,  # Let validator probability decide
            )
            trials.append((task, f"type={ctype} seed={seed}"))
    return trials


# =============================================================================
# Runner
# =============================================================================

def run_benchmark(
    trials: List[Tuple],
    run_label: str,
    verbose: bool = False,
) -> List[Dict]:
    """Run all trials sequentially with live progress output."""
    results = []
    total = len(trials)
    ts = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = RESULTS_DIR / f"benchmark_{run_label}_{ts}.csv"

    print(f"\n{'='*70}")
    print(f"  CAE BENCHMARK — {run_label}")
    print(f"  Swarm Benchmark Version: {BENCHMARK_VERSION}")
    print(f"  Total trials: {total}")
    print(f"  Horizon: {HORIZON_SEC}s | Speed limit: {SPEED_LIMIT} m/s")
    print(f"  Score weights: success={REWARD_W_SUCCESS} time={REWARD_W_TIME} safety={REWARD_W_SAFETY}")
    print(f"  Results: {csv_path}")
    print(f"{'='*70}\n")

    fieldnames = [
        'trial', 'challenge_type', 'terrain_name', 'seed', 'moving_platform',
        'horizon_s', 'success', 'collision', 'steps', 't_sim_s', 'wall_time_s',
        'min_dist_m', 'min_clearance_m', 'score', 'label',
    ]

    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for i, (task, label) in enumerate(trials):
            trial_num = i + 1
            type_name = CHALLENGE_TYPE_NAMES.get(task.challenge_type, '?')
            mp_flag   = '(MP)' if task.moving_platform else '    '
            print(
                f"[{trial_num:3d}/{total}] {type_name:<12} {mp_flag} "
                f"seed={task.map_seed:<8} ... ",
                end='', flush=True,
            )

            try:
                result = run_trial(task, verbose=verbose)
                result['trial'] = trial_num
                result['label'] = label
                results.append(result)

                status_icon = '✓' if result['success'] else ('✗' if result['collision'] else '○')
                print(
                    f"{status_icon} score={result['score']:.4f} "
                    f"t={result['t_sim_s']:.1f}s "
                    f"dist={result['min_dist_m']:.3f}m "
                    f"wall={result['wall_time_s']:.1f}s"
                )
                writer.writerow(result)
                csvfile.flush()

            except Exception as e:
                print(f"ERROR: {e}")
                err_result = {
                    'trial': trial_num, 'challenge_type': task.challenge_type,
                    'terrain_name': type_name, 'seed': task.map_seed,
                    'moving_platform': task.moving_platform,
                    'success': False, 'collision': False, 'steps': 0,
                    't_sim_s': 0.0, 'wall_time_s': 0.0,
                    'min_dist_m': 999.0, 'min_clearance_m': None,
                    'score': 0.0, 'label': label,
                }
                results.append(err_result)
                writer.writerow(err_result)
                csvfile.flush()

    print_summary(results, run_label)
    return results


def print_summary(results: List[Dict], run_label: str) -> None:
    """Print a comprehensive summary matching the validator's per-type breakdown."""
    total = len(results)
    if total == 0:
        print("No results to summarise.")
        return

    successes  = [r for r in results if r['success']]
    collisions = [r for r in results if r['collision']]
    timeouts   = [r for r in results if not r['success'] and not r['collision'] and r['steps'] > 0]
    errors     = [r for r in results if r['steps'] == 0]

    all_scores = [r['score'] for r in results]

    print(f"\n{'='*70}")
    print(f"  BENCHMARK SUMMARY — {run_label}")
    print(f"{'='*70}")
    print(f"  Total trials  : {total}")
    print(f"  Successes     : {len(successes):3d}  ({100*len(successes)/total:.1f}%)")
    print(f"  Collisions    : {len(collisions):3d}  ({100*len(collisions)/total:.1f}%)")
    print(f"  Timeouts      : {len(timeouts):3d}  ({100*len(timeouts)/total:.1f}%)")
    if errors:
        print(f"  Errors        : {len(errors):3d}")
    print(f"\n  Mean score    : {np.mean(all_scores):.4f}")
    print(f"  Median score  : {np.median(all_scores):.4f}")
    print(f"  Std score     : {np.std(all_scores):.4f}")
    if successes:
        succ_scores = [r['score'] for r in successes]
        succ_times  = [r['t_sim_s'] for r in successes]
        print(f"\n  Success mean score : {np.mean(succ_scores):.4f}")
        print(f"  Success mean time  : {np.mean(succ_times):.1f}s")

    # Per-type breakdown
    print(f"\n  {'Type':<14} {'N':>4} {'Succ':>5} {'Rate':>6} {'MeanScore':>10} {'MeanTime':>9}")
    print(f"  {'-'*55}")
    for ctype in sorted(CHALLENGE_TYPE_NAMES.keys()):
        type_results = [r for r in results if r['challenge_type'] == ctype]
        if not type_results:
            continue
        n = len(type_results)
        s = sum(1 for r in type_results if r['success'])
        mean_score = np.mean([r['score'] for r in type_results])
        succ_times = [r['t_sim_s'] for r in type_results if r['success']]
        mean_time  = np.mean(succ_times) if succ_times else float('nan')
        name = CHALLENGE_TYPE_NAMES[ctype]
        print(
            f"  {name:<14} {n:>4} {s:>5} {100*s/n:>5.1f}% "
            f"{mean_score:>10.4f} {mean_time:>8.1f}s"
        )

    print(f"{'='*70}\n")


# =============================================================================
# Entry point
# =============================================================================

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Faithful CAE benchmark — mirrors Swarm validator pipeline exactly.'
    )
    parser.add_argument(
        '--mode', choices=['screening', 'full', 'typed', 'quick'],
        default='quick',
        help=(
            'screening: use SCREENING_TEMPLATE (50-slot, no moving platforms); '
            'full: use random_task() for all seeds; '
            'typed: explicit types with --types; '
            'quick: 3 seeds × all 6 types (default)'
        ),
    )
    parser.add_argument(
        '--seeds', type=int, default=3,
        help='Number of seeds per type (quick/typed) or total seeds (screening/full)',
    )
    parser.add_argument(
        '--types', type=str, default='1,2,3,4,5,6',
        help='Comma-separated challenge types (used with --mode typed)',
    )
    parser.add_argument(
        '--seed-start', type=int, default=1000,
        help='Starting seed value (seeds are seed_start, seed_start+1, ...)',
    )
    parser.add_argument(
        '--verbose', action='store_true',
        help='Print per-100-step progress for each trial',
    )
    args = parser.parse_args()

    seeds = list(range(args.seed_start, args.seed_start + args.seeds))
    challenge_types = [int(t) for t in args.types.split(',')]

    if args.mode == 'screening':
        trials = build_screening_trials(seeds)
        label  = f'screening_{len(seeds)}seeds'
    elif args.mode == 'full':
        trials = build_random_trials(seeds)
        label  = f'full_{len(seeds)}seeds'
    elif args.mode == 'typed':
        trials = build_typed_trials(seeds, challenge_types)
        label  = f'typed_types{args.types}_{len(seeds)}seeds'
    else:  # quick
        # 3 seeds × 6 types = 18 trials using screening template slots
        quick_seeds = list(range(args.seed_start, args.seed_start + args.seeds))
        trials = build_typed_trials(quick_seeds, challenge_types)
        label  = f'quick_{len(quick_seeds)}seeds_x{len(challenge_types)}types'

    run_benchmark(trials, run_label=label, verbose=args.verbose)
