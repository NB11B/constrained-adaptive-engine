"""
benchmark_faithful.py — Faithful Swarm Subnet 142 Benchmark Harness
=====================================================================
Mirrors the validator pipeline exactly as implemented in swarm-subnet 4.0.2.9.5.

Verified against:
  swarm/utils/env_factory.py   : make_env_with_initial_obs(task, gui=False)
                                 returns (env, obs) where obs = {"depth": (128,128,1), "state": (141,)}
  swarm/validator/task_gen.py  : task_for_seed_and_type(sim_dt, seed=, challenge_type=)
                                 task.goal, task.start, task.map_seed, task.moving_platform
  swarm/validator/reward.py    : flight_reward(success, t, horizon, task, min_clearance=, collision=)
  swarm/constants.py           : SIM_DT=0.02, SPEED_LIMIT=3.0, HORIZON_SEC=60
  env.step()                   : returns (obs, reward, terminated, truncated, info)
  info keys                    : distance_to_goal, score, success, collision,
                                 t_to_goal, min_clearance, landing_stable_time
  drone_agent.py               : DroneFlightController.reset() + .act(obs) -> (5,) float32

Usage:
  python3 benchmark_faithful.py --mode typed --types 2 --seeds 5 --seed-start 39259
  python3 benchmark_faithful.py --mode screening --seeds 10 --seed-start 1000
  python3 benchmark_faithful.py --mode random --seeds 20 --seed-start 0

Copyright: Nathanael J. Bocker, 2026 all rights reserved
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Path setup
# ---------------------------------------------------------------------------
sys.path.insert(0, '/home/ubuntu/swarm-subnet')
sys.path.insert(0, '/home/ubuntu/constrained-adaptive-engine')

# ---------------------------------------------------------------------------
# Swarm imports (verified against swarm-subnet 4.0.2.9.5)
# ---------------------------------------------------------------------------
from swarm.constants import (
    BENCHMARK_TOTAL_SEED_COUNT,
    BENCHMARK_VERSION,
    CHALLENGE_TYPE_DISTRIBUTION,
    HORIZON_SEC,
    MOVING_PLATFORM_PROB,
    REWARD_W_SAFETY,
    REWARD_W_SUCCESS,
    REWARD_W_TIME,
    SCREENING_TEMPLATE,
    SIM_DT,
    SPEED_LIMIT,
)
from swarm.utils.env_factory import make_env_with_initial_obs
from swarm.validator.reward import flight_reward
from swarm.validator.task_gen import random_task, screening_task, task_for_seed_and_type

# ---------------------------------------------------------------------------
# CAE agent (DroneFlightController interface)
# ---------------------------------------------------------------------------
from drone_agent import DroneFlightController

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
CHALLENGE_TYPE_NAMES = {
    1: "City",
    2: "Open/Valley",
    3: "Mountain",
    4: "Village",
    5: "Warehouse",
    6: "Forest",
}
SCREENING_TEMPLATE_LEN = len(SCREENING_TEMPLATE)  # 50

RESULTS_DIR = Path(__file__).parent / "benchmark_results"
RESULTS_DIR.mkdir(exist_ok=True)


# =============================================================================
# Core trial runner — mirrors rpc.py evaluate loop exactly
# =============================================================================

def run_trial(
    task,
    agent: DroneFlightController,
    verbose: bool = False,
) -> Dict:
    """
    Run one evaluation trial against the real Swarm environment.

    Loop contract (mirrors rpc.py):
      - env.step(act[None, :]) — action shape (1, 5)
      - Loop while t_sim < task.horizon AND not (terminated or truncated)
      - success/collision/min_clearance from info dict
      - score from flight_reward()
    """
    wall_start = time.time()

    env, obs = make_env_with_initial_obs(task, gui=False)
    agent.reset()

    # Action space bounds (mirrors rpc.py lo/hi clip)
    lo = env.action_space.low.flatten()
    hi = env.action_space.high.flatten()

    t_sim         = 0.0
    step_count    = 0
    success       = False
    collision     = False
    min_clearance = float('inf')
    t_to_goal     = None
    info          = {}

    while t_sim < task.horizon:
        # Agent computes action from observation dict
        action = agent.act(obs)  # (5,) float32

        # nan/inf guard
        action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)

        # Clip to action space bounds (exact rpc.py)
        action = np.clip(action, lo, hi).astype(np.float32)

        # Step environment — must be (1, 5) for single-drone env
        obs, _reward, terminated, truncated, info = env.step(action[None, :])

        # Track minimum clearance
        mc = info.get('min_clearance', None)
        if mc is not None and mc < min_clearance:
            min_clearance = mc

        # Check terminal state
        success   = bool(info.get('success', False))
        collision = bool(info.get('collision', False))
        if info.get('t_to_goal') is not None:
            t_to_goal = float(info['t_to_goal'])

        t_sim      += task.sim_dt
        step_count += 1

        if verbose and step_count % 100 == 0:
            dist = info.get('distance_to_goal', float('nan'))
            print(
                f"    step={step_count:4d} t={t_sim:.1f}s "
                f"dist={dist:.3f}m "
                f"success={success} collision={collision}"
            )

        if terminated or truncated:
            break

    env.close()

    wall_time = time.time() - wall_start

    # Use t_to_goal for score if available (more precise than t_sim)
    t_flight = t_to_goal if (success and t_to_goal is not None) else t_sim
    min_cl   = min_clearance if min_clearance < float('inf') else None

    # Score (exact flight_reward call from rpc.py)
    score = flight_reward(
        success          = success,
        t                = t_flight,
        horizon          = task.horizon,
        task             = task,
        min_clearance    = min_cl,
        collision        = collision,
        legitimate_model = True,
    )

    dist_final = info.get('distance_to_goal', float('nan'))

    return {
        'challenge_type':   int(task.challenge_type),
        'terrain_name':     CHALLENGE_TYPE_NAMES.get(task.challenge_type, f'type{task.challenge_type}'),
        'seed':             int(task.map_seed),
        'moving_platform':  bool(task.moving_platform),
        'horizon_s':        float(task.horizon),
        'success':          success,
        'collision':        collision,
        'steps':            step_count,
        't_sim_s':          round(t_sim, 3),
        't_flight_s':       round(t_flight, 3),
        'wall_time_s':      round(wall_time, 2),
        'min_clearance_m':  round(min_cl, 4) if min_cl is not None else None,
        'dist_to_goal_m':   round(dist_final, 4) if dist_final == dist_final else None,
        'score':            round(score, 6),
    }


# =============================================================================
# Trial builders
# =============================================================================

def build_screening_trials(seeds: List[int]) -> List[Tuple]:
    """
    Build (task, label) pairs using the SCREENING_TEMPLATE, exactly as the
    validator does for the first BENCHMARK_TOTAL_SEED_COUNT screening seeds.
    """
    template = (
        SCREENING_TEMPLATE * ((len(seeds) // SCREENING_TEMPLATE_LEN) + 1)
    )[:len(seeds)]
    trials = []
    for i, seed in enumerate(seeds):
        slot = template[i]
        task = screening_task(
            SIM_DT,
            seed,
            challenge_type    = slot['challenge_type'],
            distance_range    = slot['distance_range'],
            goal_height_range = slot.get('goal_height_range'),
            moving_platform   = slot['moving_platform'],
        )
        label = (
            f"screening[{i % SCREENING_TEMPLATE_LEN}] "
            f"type={slot['challenge_type']} seed={seed}"
        )
        trials.append((task, label))
    return trials


def build_random_trials(seeds: List[int]) -> List[Tuple]:
    """
    Build (task, label) pairs using random_task(), exactly as the validator
    does for seeds beyond the screening range.
    """
    trials = []
    for seed in seeds:
        task  = random_task(sim_dt=SIM_DT, seed=seed)
        label = f"random type={task.challenge_type} seed={seed}"
        trials.append((task, label))
    return trials


def build_typed_trials(seeds: List[int], challenge_types: List[int]) -> List[Tuple]:
    """
    Build (task, label) pairs for explicit challenge types.
    Moving platform probability matches validator constants (moving_platform=None
    lets the task generator apply the standard probability).
    """
    trials = []
    for ctype in challenge_types:
        for seed in seeds:
            task = task_for_seed_and_type(
                SIM_DT,
                seed            = seed,
                challenge_type  = ctype,
                moving_platform = None,  # validator probability
            )
            mp_tag = ' (MP)' if task.moving_platform else ''
            label  = f"type={ctype}{mp_tag} seed={seed}"
            trials.append((task, label))
    return trials


# =============================================================================
# Benchmark runner
# =============================================================================

def run_benchmark(
    trials: List[Tuple],
    run_label: str,
    verbose: bool = False,
) -> List[Dict]:
    """Run all trials sequentially with live progress output and CSV logging."""
    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = RESULTS_DIR / f"benchmark_{run_label}_{ts}.csv"
    total    = len(trials)

    print(f"\n{'='*70}")
    print(f"  CAE BENCHMARK — {run_label}")
    print(f"  Swarm Version: {BENCHMARK_VERSION} | SIM_DT={SIM_DT}")
    print(f"  Total trials: {total}")
    print(f"  Horizon: {HORIZON_SEC}s | Speed limit: {SPEED_LIMIT} m/s")
    print(f"  Score weights: success={REWARD_W_SUCCESS} time={REWARD_W_TIME} safety={REWARD_W_SAFETY}")
    print(f"  Results: {csv_path}")
    print(f"{'='*70}\n")

    fieldnames = [
        'trial', 'label', 'challenge_type', 'terrain_name', 'seed',
        'moving_platform', 'horizon_s', 'success', 'collision', 'steps',
        't_sim_s', 't_flight_s', 'wall_time_s',
        'min_clearance_m', 'dist_to_goal_m', 'score',
    ]

    agent   = DroneFlightController()
    results = []

    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for i, (task, label) in enumerate(trials):
            trial_num = i + 1
            type_name = CHALLENGE_TYPE_NAMES.get(task.challenge_type, f'type{task.challenge_type}')
            mp_flag   = '(MP)' if task.moving_platform else '    '

            print(
                f"[{trial_num:3d}/{total}] {type_name:<12} {mp_flag} "
                f"seed={task.map_seed:<8} ... ",
                end='', flush=True,
            )

            try:
                result = run_trial(task, agent, verbose=verbose)
                result['trial'] = trial_num
                result['label'] = label

                status = '✓' if result['success'] else ('✗' if result['collision'] else '○')
                print(
                    f"{status} score={result['score']:.4f} "
                    f"t={result['t_flight_s']:.1f}s "
                    f"dist={result.get('dist_to_goal_m', float('nan')):.3f}m "
                    f"wall={result['wall_time_s']:.1f}s"
                )

            except Exception as e:
                print(f"ERROR: {e}")
                result = {fn: None for fn in fieldnames}
                result.update({
                    'trial':          trial_num,
                    'label':          label,
                    'challenge_type': task.challenge_type,
                    'terrain_name':   type_name,
                    'seed':           task.map_seed,
                    'moving_platform':task.moving_platform,
                    'horizon_s':      float(task.horizon),
                    'success':        False,
                    'collision':      False,
                    'steps':          0,
                    'score':          0.0,
                })

            results.append(result)
            writer.writerow(result)
            csvfile.flush()

    _print_summary(results, run_label)
    return results


def _print_summary(results: List[Dict], run_label: str) -> None:
    """Print a comprehensive per-type summary."""
    total = len(results)
    if total == 0:
        print("No results to summarise.")
        return

    n_success   = sum(1 for r in results if r.get('success'))
    n_collision = sum(1 for r in results if r.get('collision'))
    n_timeout   = sum(1 for r in results if not r.get('success') and not r.get('collision') and r.get('steps', 0) > 0)
    all_scores  = [r['score'] for r in results if r.get('score') is not None]

    print(f"\n{'='*70}")
    print(f"  BENCHMARK SUMMARY — {run_label}")
    print(f"{'='*70}")
    print(f"  Total trials  : {total}")
    print(f"  Successes     : {n_success:3d}  ({100*n_success/total:.1f}%)")
    print(f"  Collisions    : {n_collision:3d}  ({100*n_collision/total:.1f}%)")
    print(f"  Timeouts      : {n_timeout:3d}  ({100*n_timeout/total:.1f}%)")
    if all_scores:
        print(f"\n  Mean score    : {np.mean(all_scores):.4f}")
        print(f"  Median score  : {np.median(all_scores):.4f}")
        print(f"  Std score     : {np.std(all_scores):.4f}")

    succ_results = [r for r in results if r.get('success')]
    if succ_results:
        succ_scores = [r['score'] for r in succ_results]
        succ_times  = [r['t_flight_s'] for r in succ_results if r.get('t_flight_s') is not None]
        print(f"\n  Success mean score : {np.mean(succ_scores):.4f}")
        if succ_times:
            print(f"  Success mean time  : {np.mean(succ_times):.1f}s")

    # Per-type breakdown
    print(f"\n  {'Type':<14} {'N':>4} {'Succ':>5} {'Rate':>6} {'MeanScore':>10} {'MeanTime':>9}")
    print(f"  {'-'*55}")
    for ctype in sorted(CHALLENGE_TYPE_NAMES.keys()):
        type_results = [r for r in results if r.get('challenge_type') == ctype]
        if not type_results:
            continue
        n      = len(type_results)
        s      = sum(1 for r in type_results if r.get('success'))
        scores = [r['score'] for r in type_results if r.get('score') is not None]
        times  = [r['t_flight_s'] for r in type_results if r.get('success') and r.get('t_flight_s') is not None]
        mean_score = np.mean(scores) if scores else float('nan')
        mean_time  = np.mean(times) if times else float('nan')
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
        '--mode', choices=['screening', 'random', 'typed', 'quick'],
        default='quick',
        help=(
            'screening: use SCREENING_TEMPLATE (50-slot, no moving platforms); '
            'random: use random_task() for all seeds; '
            'typed: explicit types with --types; '
            'quick: 3 seeds × 6 types (default)'
        ),
    )
    parser.add_argument(
        '--seeds', type=int, default=3,
        help='Number of seeds per type (typed/quick) or total seeds (screening/random)',
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

    seeds           = list(range(args.seed_start, args.seed_start + args.seeds))
    challenge_types = [int(t) for t in args.types.split(',')]

    if args.mode == 'screening':
        trials = build_screening_trials(seeds)
        label  = f'screening_{len(seeds)}seeds'
    elif args.mode == 'random':
        trials = build_random_trials(seeds)
        label  = f'random_{len(seeds)}seeds'
    elif args.mode == 'typed':
        trials = build_typed_trials(seeds, challenge_types)
        label  = f'typed_types{"".join(str(t) for t in challenge_types)}_{len(seeds)}seeds'
    else:  # quick
        trials = build_typed_trials(seeds, challenge_types)
        label  = f'quick_{len(seeds)}seeds_x{len(challenge_types)}types'

    run_benchmark(trials, run_label=label, verbose=args.verbose)
