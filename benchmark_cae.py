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
            f"benchmark_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
        )
        self.summary = []

    def run_benchmark(self, num_trials_per_terrain=5, random_seed_range=(0, 10000), use_fixed_seeds=False):
        print("=" * 60)
        print(
            f"CAE Real Swarm Benchmarker | Trials per Terrain: {num_trials_per_terrain} | "
            f"Random: {not use_fixed_seeds}"
        )
        print("=" * 60)

        fixed_seeds = [1337, 42, 6969, 9328, 2062, 7812, 101, 202, 303, 404]

        with open(self.results_file, mode="w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "Terrain",
                "Terrain ID",
                "Seed",
                "Status",
                "Steps",
                "Time (s)",
                "Min Dist (m)",
                "Landing Phase",
                "Descent Phase",
            ])

            for terrain_id, terrain_name in TERRAIN_NAMES.items():
                print(f"\nBenchmarking Terrain: {terrain_name}")
                for i in range(num_trials_per_terrain):
                    if use_fixed_seeds:
                        seed = fixed_seeds[i % len(fixed_seeds)]
                    else:
                        seed = random.randint(*random_seed_range)

                    print(f"  Trial {i + 1}/{num_trials_per_terrain} | Seed: {seed}...", end="", flush=True)
                    t0 = time.time()
                    result = run_real_env_trial(terrain_id, seed, max_steps=3000, verbose=False)
                    elapsed = time.time() - t0
                    result["wall_time_s"] = elapsed

                    writer.writerow([
                        terrain_name,
                        terrain_id,
                        seed,
                        result["status"],
                        result["steps"],
                        f"{elapsed:.2f}",
                        f"{result['min_dist_m']:.3f}",
                        result.get("landing_phase", False),
                        result.get("descent_phase", False),
                    ])
                    f.flush()

                    print(
                        f" {result['status']} "
                        f"({result['steps']} steps, min_dist={result['min_dist_m']}m, "
                        f"landing={result.get('landing_phase', False)}, "
                        f"descent={result.get('descent_phase', False)}, wall={elapsed:.2f}s)"
                    )
                    self.summary.append(result)

        self.print_final_summary()

    def print_final_summary(self):
        print("\n" + "=" * 60)
        print("BENCHMARK SUMMARY")
        print("=" * 60)

        successes = [r for r in self.summary if r["status"] == "SUCCESS"]
        collisions = [r for r in self.summary if r["status"] == "COLLISION"]
        timeouts = [r for r in self.summary if r["status"] == "TIMEOUT"]
        errors = [r for r in self.summary if "ERROR" in r["status"]]
        landing_entries = [r for r in self.summary if r.get("landing_phase", False)]
        descent_entries = [r for r in self.summary if r.get("descent_phase", False)]

        total = len(self.summary)
        success_rate = (len(successes) / total * 100) if total > 0 else 0.0

        print(f"Total Trials: {total}")
        print(f"Successes:    {len(successes)} ({success_rate:.1f}%)")
        print(f"Collisions:   {len(collisions)}")
        print(f"Timeouts:     {len(timeouts)}")
        print(f"Landing Seen: {len(landing_entries)}")
        print(f"Descent Seen: {len(descent_entries)}")
        if errors:
            print(f"Errors:       {len(errors)}")

        if successes:
            avg_steps = np.mean([r["steps"] for r in successes])
            avg_wall = np.mean([r["wall_time_s"] for r in successes])
            print(f"Avg Steps (Success): {avg_steps:.1f}")
            print(f"Avg Wall Time:       {avg_wall:.2f}s")

        print(f"\nDetailed results saved to: {self.results_file}")
        print("=" * 60)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="CAE real Swarm benchmark runner")
    parser.add_argument("--trials", type=int, default=2, help="Number of trials per terrain")
    parser.add_argument("--fixed", action="store_true", help="Use fixed seeds for reproducibility")
    parser.add_argument("--seed-min", type=int, default=0, help="Minimum random seed")
    parser.add_argument("--seed-max", type=int, default=10000, help="Maximum random seed")
    args = parser.parse_args()

    benchmarker = CAEBenchmarker()
    benchmarker.run_benchmark(
        num_trials_per_terrain=args.trials,
        random_seed_range=(args.seed_min, args.seed_max),
        use_fixed_seeds=args.fixed,
    )
