"""
diagnose_trial.py — Trace first 10 steps of a type-2 trial to find early termination cause.
"""
import sys
sys.path.insert(0, '/home/ubuntu/swarm-subnet')
sys.path.insert(0, '/home/ubuntu/constrained-adaptive-engine')

import numpy as np
from swarm.constants import SIM_DT
from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env_with_initial_obs
from drone_agent import DroneFlightController

task = task_for_seed_and_type(SIM_DT, seed=39259, challenge_type=2)
print(f"Task: type={task.challenge_type} seed={task.map_seed} mp={task.moving_platform}")
print(f"  goal={task.goal}")
print(f"  start={task.start}")
print(f"  horizon={task.horizon}")

env, obs = make_env_with_initial_obs(task, gui=False)
agent = DroneFlightController()
agent.reset()

lo = env.action_space.low.flatten()
hi = env.action_space.high.flatten()

print(f"\nObs keys: {list(obs.keys())}")
print(f"State shape: {obs['state'].shape}")
print(f"Depth shape: {obs['depth'].shape}")
print(f"Initial pos: {obs['state'][0:3]}")
print(f"Initial vel: {obs['state'][6:9]}")
print(f"Action space lo: {lo}")
print(f"Action space hi: {hi}")

t_sim = 0.0
for step in range(20):
    action = agent.act(obs)
    action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)
    action_clipped = np.clip(action, lo, hi).astype(np.float32)

    obs, reward, terminated, truncated, info = env.step(action_clipped[None, :])
    t_sim += task.sim_dt

    pos = obs['state'][0:3]
    vel = obs['state'][6:9]
    print(
        f"  step={step+1:3d} t={t_sim:.2f}s "
        f"pos=[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] "
        f"vel=[{vel[0]:.2f},{vel[1]:.2f},{vel[2]:.2f}] "
        f"action={action_clipped} "
        f"reward={reward:.4f} "
        f"term={terminated} trunc={truncated} "
        f"info={info}"
    )

    if terminated or truncated:
        print(f"\n*** TERMINATED at step {step+1} ***")
        print(f"  success={info.get('success')} collision={info.get('collision')}")
        break

env.close()
