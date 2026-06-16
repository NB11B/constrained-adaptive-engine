"""
inspect_state.py — Inspect the 141-dim state vector layout in swarm-subnet 4.0.2.9.5
"""
import sys
sys.path.insert(0, '/home/ubuntu/swarm-subnet')
sys.path.insert(0, '/home/ubuntu/constrained-adaptive-engine')

import numpy as np
from swarm.constants import SIM_DT
from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env_with_initial_obs

task = task_for_seed_and_type(SIM_DT, seed=39259, challenge_type=2)
print(f"Task goal: {task.goal}")
print(f"Task start: {task.start}")
print(f"Expected goal-start vector: {np.array(task.goal) - np.array(task.start)}")

env, obs = make_env_with_initial_obs(task, gui=False)
state = obs['state']

print(f"\nState vector (141 dims):")
print(f"  [0:3]   pos_xyz = {state[0:3]}")
print(f"  [3:6]   rpy     = {state[3:6]}")
print(f"  [6:9]   vel_xyz = {state[6:9]}")
print(f"  [9:12]  ang_vel = {state[9:12]}")
print(f"  [12:16] action_hist[0] = {state[12:16]}")
print(f"  [108:112] action_hist[-1] = {state[108:112]}")
print(f"  [112]   state[112] = {state[112]}")
print(f"  [113:116] state[113:116] = {state[113:116]}")
print(f"  [116:120] state[116:120] = {state[116:120]}")
print(f"  [120:124] state[120:124] = {state[120:124]}")
print(f"  [124:128] state[124:128] = {state[124:128]}")
print(f"  [128:132] state[128:132] = {state[128:132]}")
print(f"  [132:136] state[132:136] = {state[132:136]}")
print(f"  [136:141] state[136:141] = {state[136:141]}")

# Compute what the goal direction should be
pos = state[0:3]
goal = np.array(task.goal)
goal_vec = goal - pos
print(f"\nExpected goal-pos vector: {goal_vec}")
print(f"state[113:116] = {state[113:116]}")
print(f"Ratio (state[113:116] / goal_vec) = {state[113:116] / (goal_vec + 1e-10)}")

# Check if state[112] is AGL
print(f"\nstate[112] = {state[112]}")
print(f"pos[2] = {pos[2]}")
print(f"If AGL_norm: AGL_m = {state[112] * 20.0}")

# Check if state[113:116] is normalized search_vec
norm = np.linalg.norm(state[113:116])
print(f"\n|state[113:116]| = {norm}")
print(f"|goal_vec| = {np.linalg.norm(goal_vec)}")
print(f"If normalized: state[113:116] * |goal_vec| = {state[113:116] * np.linalg.norm(goal_vec)}")

# Check the search_radius
print(f"\ntask.search_radius = {task.search_radius}")
print(f"If search_vec/search_radius: {state[113:116] * task.search_radius}")

env.close()
