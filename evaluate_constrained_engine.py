import sys
import os
import time
import numpy as np

# Add the crazyflow_swarm_sim directory to sys.path
sys.path.append('/home/ubuntu/crazyflow_swarm_sim')

try:
    from evaluate_sotapilot import ProcScene, TERRAIN_CONFIGS as TERRAINS
except ImportError:
    print("Error: Could not import ProcScene or TERRAINS from evaluate_sotapilot.py.")
    print("Ensure that crazyflow_swarm_sim repository is cloned and in the correct path.")
    sys.exit(1)

from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine

def run_trial(engine, terrain_id, seed=0, max_steps=1500):
    cfg = TERRAINS[terrain_id]
    scene = ProcScene(cfg, seed)
    
    # Reset engine for new trial
    # (Assuming we might want a fresh state, but let's just start adaptive)
    engine.set_flight_params(
        max_speed=3.0,
        safety_radius=0.7,
        mode_v=1.5,
        platform_vel_est=[0.0, 0.0, 0.0]
    )
    engine.set_target_pos(scene.pad_pos)
    engine.start_adaptive()

    success = False
    collision = False
    
    # Initial observation
    obs_dict, _ = scene.reset()
    
    start_time = time.time()
    
    for step in range(max_steps):
        state_vec = obs_dict["state"]
        # state_vec: [pos(3), rpy(3), vel(3), ..., alt(-4), rel_target(-3:)]
        current_pos = state_vec[0:3]
        current_rpy = state_vec[3:6]
        current_vel = state_vec[6:9]
        target_pos = scene.pad_pos # Use ground truth from scene
        yaw_rate = 0.0 # Not explicitly in state_vec, but used by C engine for damping
        agl = state_vec[-4] * 20.0 # Unscale altitude
        
        # Prepare obstacles for C engine: [x, y, h, r] -> [x, y, z, r]
        # ProcScene stores obstacles as (ox, oy, oh, orad)
        obstacles = []
        for ox, oy, oh, orad in scene.obstacles:
            # The C engine treats obstacles as spheres/points at (ox, oy, oz) with radius or
            # For cylinders, we can approximate by placing a point at the drone's altitude
            # but constrained within [0, oh]
            oz = np.clip(current_pos[2], 0, oh)
            obstacles.append([ox, oy, oz, orad])
        
        num_obstacles = len(obstacles)
        # Flatten and pad for the bridge (bridge handles flattening if we pass list of lists)
        
        # Process sensor data and update engine
        engine.process_sensor_data(
            current_pos, current_vel, current_rpy, target_pos, 
            yaw_rate, agl, obstacles, num_obstacles
        )
        engine.update()
        
        # Get control output from engine
        state = engine.get_state()
        # control_output: [vx, vy, vz, total_speed, yaw_rate_cmd]
        vx, vy, vz, total_speed, yaw_cmd = state.control_output
        
        # Format action for ProcScene.step: [dir_x, dir_y, dir_z, speed_norm, yaw_norm]
        if total_speed > 1e-6:
            dir_xyz = [vx/total_speed, vy/total_speed, vz/total_speed]
        else:
            dir_xyz = [1.0, 0.0, 0.0]
            
        speed_norm = total_speed / 3.0 # ProcScene scales by 3.0
        action = np.array([dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_cmd])
        
        # Step the simulator
        obs_dict, reward, terminated, truncated, info = scene.step(action)
        
        if info.get('collision', False):
            collision = True
            break
        if info.get('success', False):
            success = True
            break
        if terminated or truncated:
            break
            
    end_time = time.time()
    duration = end_time - start_time
    
    # No scene.close() needed as per evaluate_sotapilot.py
    
    return {
        'success': success,
        'collision': collision,
        'timeout': not (success or collision),
        'steps': step + 1,
        'duration': duration,
        'terrain': cfg["name"],
        'seed': seed
    }

def main():
    engine = ConstrainedAdaptiveEngine()
    print("Constrained Adaptive Engine loaded.")

    terrain_ids = list(TERRAINS.keys())
    seeds = [0, 1, 2] # 3 seeds per terrain as per SOTAPilot report

    results = []

    for terrain_id in terrain_ids:
        for seed in seeds:
            terrain_name = TERRAINS[terrain_id]["name"]
            print(f"Running Trial: Terrain={terrain_name}, Seed={seed}...", end='', flush=True)
            result = run_trial(engine, terrain_id, seed)
            results.append(result)
            status = "SUCCESS" if result['success'] else ("COLLISION" if result['collision'] else "TIMEOUT")
            print(f" {status} ({result['steps']} steps, {result['duration']:.2f}s)")

    # Print Summary Table
    print("\n" + "="*60)
    print(f"{'Terrain':<15} | {'Seed':<10} | {'Status':<10} | {'Steps':<6}")
    print("-" * 60)
    for r in results:
        status = "SUCCESS" if r['success'] else ("COLLISION" if r['collision'] else "TIMEOUT")
        print(f"{r['terrain']:<15} | {r['seed']:<10} | {status:<10} | {r['steps']:<6}")
    print("="*60)

    success_count = sum(1 for r in results if r['success'])
    collision_count = sum(1 for r in results if r['collision'])
    timeout_count = sum(1 for r in results if r['timeout'])
    
    print(f"Overall Success Rate: {success_count}/{len(results)} ({success_count/len(results)*100:.1f}%)")
    print(f"Collisions: {collision_count}, Timeouts: {timeout_count}")

if __name__ == '__main__':
    main()
