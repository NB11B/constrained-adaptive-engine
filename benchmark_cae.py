import os
import sys
import time
import random
import csv
import numpy as np
from datetime import datetime

# Add parent directory and crazyflow-swarm-sim to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../crazyflow_swarm_sim')))

from evaluate_constrained_engine import TERRAINS
from evaluate_sotapilot import ProcScene
from constrained_adaptive_engine_bridge import ConstrainedAdaptiveEngine

class SOTAPilotBenchmarker:
    def __init__(self, output_dir="benchmarks"):
        self.output_dir = output_dir
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
        
        self.results_file = os.path.join(output_dir, f"benchmark_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv")
        self.summary = []

    def run_benchmark(self, num_trials_per_terrain=5, random_seed_range=(0, 10000), use_fixed_seeds=False):
        print("="*60)
        print(f"SOTAPilot CAE Benchmarker | Trials per Terrain: {num_trials_per_terrain} | Random: {not use_fixed_seeds}")
        print("="*60)

        fixed_seeds = [1337, 42, 6969, 9328, 2062, 7812, 101, 202, 303, 404]

        with open(self.results_file, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(["Terrain", "Seed", "Status", "Steps", "Time (s)", "Avg Latency (ms)"])

            for terrain_id, terrain_cfg in TERRAINS.items():
                terrain_name = terrain_cfg["name"]
                print(f"\nBenchmarking Terrain: {terrain_name}")
                for i in range(num_trials_per_terrain):
                    if use_fixed_seeds:
                        seed = fixed_seeds[i % len(fixed_seeds)]
                    else:
                        seed = random.randint(*random_seed_range)
                    print(f"  Trial {i+1}/{num_trials_per_terrain} | Seed: {seed}...", end="", flush=True)
                    
                    result = self.run_single_trial(terrain_id, seed)
                    
                    writer.writerow([
                        terrain_name, seed, result['status'], 
                        result['steps'], f"{result['time']:.2f}", 
                        f"{result['avg_latency']:.2f}"
                    ])
                    f.flush()
                    
                    print(f" {result['status']} ({result['steps']} steps, {result['time']:.2f}s)")
                    self.summary.append(result)

        self.print_final_summary()

    def run_single_trial(self, terrain_id, seed, max_steps=1500):
        try:
            # Initialize environment and pilot
            cfg = TERRAINS[terrain_id]
            scene = ProcScene(cfg, seed)
            engine = ConstrainedAdaptiveEngine()
            
            # Reset engine for new trial
            engine.set_flight_params(
                max_speed=5.0,
                safety_radius=1.15,
                mode_v=200.0,
                platform_vel_est=[0.0, 0.0, 0.0]
            )
            engine.set_target_pos(scene.pad_pos)
            engine.start_adaptive()
            
            # Initial observation
            obs_dict, _ = scene.reset()
            
            start_time = time.time()
            steps = 0
            latencies = []
            
            while steps < max_steps:
                step_start = time.time()
                
                state_vec = obs_dict["state"]
                current_pos = state_vec[0:3]
                current_rpy = state_vec[3:6]
                current_vel = state_vec[6:9]
                target_pos = scene.pad_pos
                yaw_rate = 0.0
                agl = state_vec[-4] * 20.0
                
                obstacles = []
                for ox, oy, oh, orad in scene.obstacles:
                    oz = np.clip(current_pos[2], 0, oh)
                    obstacles.append([ox, oy, oz, orad])
                
                # Update engine
                engine.process_sensor_data(
                    current_pos, current_vel, current_rpy, target_pos, 
                    yaw_rate, agl, obstacles)
                engine.update()
                
                # Get control output
                state = engine.get_state()
                vx, vy, vz, total_speed, yaw_cmd = state.control_output
                
                if total_speed > 1e-6:
                    dir_xyz = [vx/total_speed, vy/total_speed, vz/total_speed]
                else:
                    dir_xyz = [1.0, 0.0, 0.0]
                    
                speed_norm = total_speed / 3.0 # ProcScene scales by 3.0
                action = np.array([dir_xyz[0], dir_xyz[1], dir_xyz[2], speed_norm, yaw_cmd])
                
                # Step the simulator
                obs_dict, reward, terminated, truncated, info = scene.step(action)
                
                latencies.append((time.time() - step_start) * 1000)
                steps += 1
                
                if info.get('collision', False):
                    status = "COLLISION"
                    break
                if info.get('success', False):
                    status = "SUCCESS"
                    break
                if terminated or truncated:
                    status = "TERMINATED"
                    break
            else:
                status = "TIMEOUT"
            
            total_time = time.time() - start_time
            avg_latency = np.mean(latencies) if latencies else 0
            
            return {
                "status": status,
                "steps": steps,
                "time": total_time,
                "avg_latency": avg_latency
            }
        except Exception as e:
            return {
                "status": f"ERROR: {str(e)}",
                "steps": 0,
                "time": 0,
                "avg_latency": 0
            }

    def print_final_summary(self):
        print("\n" + "="*60)
        print("BENCHMARK SUMMARY")
        print("="*60)
        
        successes = [r for r in self.summary if r['status'] == "SUCCESS"]
        collisions = [r for r in self.summary if r['status'] == "COLLISION"]
        timeouts = [r for r in self.summary if r['status'] == "TIMEOUT"]
        errors = [r for r in self.summary if "ERROR" in r['status']]
        
        total = len(self.summary)
        success_rate = (len(successes) / total * 100) if total > 0 else 0
        
        print(f"Total Trials: {total}")
        print(f"Successes:    {len(successes)} ({success_rate:.1f}%)")
        print(f"Collisions:   {len(collisions)}")
        print(f"Timeouts:     {len(timeouts)}")
        if errors:
            print(f"Errors:       {len(errors)}")
        
        if successes:
            avg_steps = np.mean([r['steps'] for r in successes])
            avg_latency = np.mean([r['avg_latency'] for r in successes])
            print(f"Avg Steps (Success):   {avg_steps:.1f}")
            print(f"Avg Latency (Success): {avg_latency:.2f} ms")
        
        print(f"\nDetailed results saved to: {self.results_file}")
        print("="*60)

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="SOTAPilot CAE Benchmarker")
    parser.add_argument("--trials", type=int, default=2, help="Number of trials per terrain")
    parser.add_argument("--fixed", action="store_true", help="Use fixed seeds for reproducibility")
    args = parser.parse_args()

    benchmarker = SOTAPilotBenchmarker()
    # Run benchmark with specified options
    benchmarker.run_benchmark(num_trials_per_terrain=args.trials, use_fixed_seeds=args.fixed)
