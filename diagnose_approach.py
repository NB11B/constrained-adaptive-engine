"""
diagnose_approach.py — Trace the final approach to understand collision cause.
"""
import sys
sys.path.insert(0, '/home/ubuntu/swarm-subnet')
sys.path.insert(0, '/home/ubuntu/constrained-adaptive-engine')

import numpy as np
from swarm.constants import SIM_DT
from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env_with_initial_obs
from drone_agent import DroneFlightController

# Use seed 39261 which gets closest (dist=0.836m) before collision
task = task_for_seed_and_type(SIM_DT, seed=39261, challenge_type=2)
print(f"Task: type={task.challenge_type} seed={task.map_seed} mp={task.moving_platform}")
print(f"  goal={task.goal}")
print(f"  start={task.start}")
print(f"  horizon={task.horizon}")

env, obs = make_env_with_initial_obs(task, gui=False)
agent = DroneFlightController()
agent.reset()

lo = env.action_space.low.flatten()
hi = env.action_space.high.flatten()

t_sim = 0.0
min_dist = 999.0
for step in range(3000):  # full 60s horizon
    action = agent.act(obs)
    action = np.nan_to_num(action, nan=0.0, posinf=0.0, neginf=0.0)
    action_clipped = np.clip(action, lo, hi).astype(np.float32)

    obs, reward, terminated, truncated, info = env.step(action_clipped[None, :])
    t_sim += task.sim_dt

    pos = obs['state'][0:3]
    vel = obs['state'][6:9]
    dist = info.get('distance_to_goal', float('nan'))
    if dist < min_dist:
        min_dist = dist

    # Print every 50 steps, or when close to goal, or on termination
    if step % 50 == 0 or dist < 5.0 or terminated or truncated:
        speed = float(np.linalg.norm(vel))
        print(
            f"  step={step+1:4d} t={t_sim:.2f}s "
            f"pos=[{pos[0]:.2f},{pos[1]:.2f},{pos[2]:.2f}] "
            f"vel=[{vel[0]:.2f},{vel[1]:.2f},{vel[2]:.2f}] spd={speed:.2f} "
            f"dist={dist:.3f}m "
            f"act=[{action_clipped[0]:.2f},{action_clipped[1]:.2f},{action_clipped[2]:.2f},{action_clipped[3]:.2f}] "
            f"term={terminated} trunc={truncated}"
        )

    if terminated or truncated:
        print(f"\n*** TERMINATED at step {step+1} t={t_sim:.2f}s ***")
        print(f"  success={info.get('success')} collision={info.get('collision')}")
        print(f"  min_dist={min_dist:.3f}m")
        print(f"  goal={task.goal}")
        break

env.close()
print(f"\nFinal min_dist: {min_dist:.3f}m")
