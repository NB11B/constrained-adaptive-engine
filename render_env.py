import sys
import os
import numpy as np
import pybullet as p
import matplotlib.pyplot as plt
from PIL import Image

# Set the SWARM_REPO path
SWARM_REPO = '/home/ubuntu/swarm-subnet'
sys.path.insert(0, SWARM_REPO)
sys.path.append('/home/ubuntu/constrained-adaptive-engine')

from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env

def render_state(terrain_id, seed, step_to_render, output_path):
    print(f"Rendering Terrain {terrain_id}, Seed {seed} at Step {step_to_render}...")
    # 1. Initialize environment
    task = task_for_seed_and_type(0.01, seed=seed, challenge_type=terrain_id)
    
    # Pass gui=False directly to make_env
    env = make_env(task, gui=False)
    
    # 2. Run until the specified step
    env.reset()
    
    drone_uid = env.unwrapped.DRONE_IDS[0]
    plat_uid = env.unwrapped._end_platform_uids[-1]
    
    for i in range(step_to_render):
        action = np.zeros((1, 5))
        step_res = env.step(action)
        if len(step_res) == 5:
            _, _, terminated, truncated, _ = step_res
            done = terminated or truncated
        else:
            _, _, done, _ = step_res
            
        if done:
            print(f"Environment finished at step {i}")
            break
            
    # 3. Get camera view
    drone_pos, _ = p.getBasePositionAndOrientation(drone_uid, physicsClientId=env.unwrapped.CLIENT)
    pad_pos_raw, _ = p.getBasePositionAndOrientation(plat_uid, physicsClientId=env.unwrapped.CLIENT)
    pad_pos = list(pad_pos_raw)
    pad_pos[2] = 0.2 # Standardize pad height
    
    print(f"Drone Pos: {drone_pos}, Pad Pos: {pad_pos}")
    
    # Overhead view focused on the pad
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[pad_pos[0], pad_pos[1], pad_pos[2] + 5],
        cameraTargetPosition=[pad_pos[0], pad_pos[1], pad_pos[2]],
        cameraUpVector=[0, 1, 0],
        physicsClientId=env.unwrapped.CLIENT
    )
    
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=1.0, nearVal=0.1, farVal=100.0,
        physicsClientId=env.unwrapped.CLIENT
    )
    
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        width=800, height=800,
        viewMatrix=view_matrix,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=env.unwrapped.CLIENT
    )
    
    # 4. Save the image
    rgb_array = np.reshape(rgb_img, (height, width, 4))
    img = Image.fromarray(rgb_array[:, :, :3])
    img.save(output_path)
    print(f"Environment render saved to {output_path}")
    
    # Side view showing drone and pad
    view_matrix_side = p.computeViewMatrix(
        cameraEyePosition=[pad_pos[0] + 3, pad_pos[1] + 3, pad_pos[2] + 3],
        cameraTargetPosition=[pad_pos[0], pad_pos[1], pad_pos[2]],
        cameraUpVector=[0, 0, 1],
        physicsClientId=env.unwrapped.CLIENT
    )
    
    width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
        width=800, height=800,
        viewMatrix=view_matrix_side,
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER,
        physicsClientId=env.unwrapped.CLIENT
    )
    
    rgb_array = np.reshape(rgb_img, (height, width, 4))
    img_side = Image.fromarray(rgb_array[:, :, :3])
    img_side.save(output_path.replace('.png', '_side.png'))
    print(f"Side render saved to {output_path.replace('.png', '_side.png')}")
    
    env.close()

if __name__ == "__main__":
    # Warehouse (5) seed 1337 at step 2284 (collision)
    render_state(5, 1337, 2284, 'warehouse_collision_render.png')
    # Mountain (3) seed 1337 at step 3000 (timeout/end)
    render_state(3, 1337, 3000, 'mountain_timeout_render.png')
