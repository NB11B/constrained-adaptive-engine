"""Phase-instrumented diagnostic: trace landing_phase and descent_phase activation."""
import sys
sys.path.insert(0, '/home/ubuntu/swarm-subnet')
sys.path.insert(0, '/home/ubuntu/constrained-adaptive-engine')

import numpy as np
from swarm.constants import SIM_DT
from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env_with_initial_obs
from drone_agent import DroneFlightController

SEED = 39261
CHALLENGE_TYPE = 2
MAX_STEPS = 3000

task = task_for_seed_and_type(SIM_DT, seed=SEED, challenge_type=CHALLENGE_TYPE)
env, obs = make_env_with_initial_obs(task, gui=False)
agent = DroneFlightController()
agent.reset()

goal = np.array(task.goal)
print(f"goal = {goal}")
print(f"spawn = {obs['state'][0:3]}")
print()

prev_landing = False
prev_descent = False

for step in range(1, MAX_STEPS + 1):
    action = agent.act(obs)
    obs, reward, terminated, truncated, info = env.step(action[None, :])

    state = obs['state']
    pos = state[0:3]
    vel = state[6:9]
    spd = np.linalg.norm(vel)
    dist = np.linalg.norm(pos - goal)
    dist_xy = np.linalg.norm(pos[:2] - goal[:2])

    # Get phase state from C engine
    cs = agent._engine.get_state()
    lp = bool(cs.landing_phase)
    dp = bool(cs.descent_phase)
    t = step * SIM_DT

    # Print when phase changes or every 50 steps
    phase_changed = (lp != prev_landing) or (dp != prev_descent)
    if phase_changed or step % 50 == 0 or step <= 5:
        print(f"  step={step:4d} t={t:.2f}s z={pos[2]:.3f} vz={vel[2]:.3f} "
              f"dist_xy={dist_xy:.3f}m dist3d={dist:.3f}m "
              f"LP={'Y' if lp else 'N'} DP={'Y' if dp else 'N'} "
              f"act=[{action[0]:.2f},{action[1]:.2f},{action[2]:.2f},{action[3]:.2f}]")

    prev_landing = lp
    prev_descent = dp

    if terminated or truncated:
        print(f"\n*** {'TERMINATED' if terminated else 'TRUNCATED'} at step {step} t={t:.2f}s ***")
        print(f"  success={info.get('success', False)} collision={info.get('collision', False)}")
        print(f"  min_dist={info.get('min_clearance', dist):.3f}m")
        break

env.close()
