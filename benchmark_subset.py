import argparse
import csv
import os
import random
import time
from datetime import datetime
import numpy as np
from test_real_env import TERRAIN_NAMES, run_real_env_trial

class CAEBenchmarker:
    def __init__(self, output_dir="benchmarks"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.results_file = os.path.join(
            output_dir,
            f"benchmark_subset_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        self.summary = []

    def run_benchmark(self, terrains, num_trials_per_terrain=1, use_fixed_seeds=True):
        print("=" * 60)
        print(f"CAE Subset Benchmarker | Terrains: {terrains}")
        print("=" * 60)
        fixed_seeds = [1337, 42, 6969, 9328, 2062, 7812, 101, 202, 303, 404]
        with open(self.results_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Terrain", "Terrain ID", "Seed", "Status", "Steps", "Time (s)", "Min Dist (m)",
                "Landing Phase", "Descent Phase", "Approach Assist Ticks", "Terminal Assist Ticks",
                "Last Phase Reason", "Min Dist Step", "Min Dist XY", "Min AGL",
                "AGL Floor Ticks", "Descent Corridor Ticks", "Settle Phase Ticks", "Terminal Press Ticks"
            ])
            for terrain_id in terrains:
                terrain_name = TERRAIN_NAMES.get(terrain_id, f"ID {terrain_id}")
                print(f"\nBenchmarking Terrain: {terrain_name}")
                for i in range(num_trials_per_terrain):
                    seed = fixed_seeds[i % len(fixed_seeds)]
                    print(f"  Trial {i + 1}/{num_trials_per_terrain} | Seed: {seed}...", end="", flush=True)
                    t0 = time.time()
                    result = run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=True)
                    elapsed = time.time() - t0
                    writer.writerow([
                        terrain_name, terrain_id, seed, result["status"], result["steps"], f"{elapsed:.2f}", f"{result['min_dist_m']:.3f}",
                        result.get("landing_phase", False), result.get("descent_phase", False),
                        result.get("approach_assist_ticks", 0), result.get("terminal_assist_ticks", 0),
                        result.get("last_phase_reason", "N/A"), result.get("min_dist_step", 0),
                        result.get("min_dist_xy", 9999.0), result.get("min_agl", 9999.0),
                        result.get("agl_floor_ticks", 0), result.get("descent_corridor_ticks", 0),
                        result.get("settle_phase_ticks", 0), result.get("terminal_press_ticks", 0)
                    ])
                    f.flush()
                    print(f" {result['status']} ({result['steps']} steps, min_dist={result['min_dist_m']}m, wall={elapsed:.2f}s)")
                    self.summary.append(result)
        print("\nDONE.")

if __name__ == "__main__":
    benchmarker = CAEBenchmarker()
    # Run only terrains 3 (Mountain), 4 (Village), 5 (Warehouse), 6 (Forest)
    # City (1) and Open (2) already succeeded in the previous partial run.
    benchmarker.run_benchmark(terrains=[3, 4, 5, 6])
