import random
import time
import argparse

def generate_seeds(count=10, seed_range=(0, 100000)):
    """Generate a list of random seeds."""
    return [random.randint(*seed_range) for _ in range(count)]

def main():
    parser = argparse.ArgumentParser(description="SOTAPilot Seed Randomizer Utility")
    parser.add_argument("--count", type=int, default=1, help="Number of seeds to generate")
    parser.add_argument("--continuous", action="store_true", help="Run in continuous mode (print seed every second)")
    args = parser.parse_args()

    if args.continuous:
        print("Starting continuous seed generation (Ctrl+C to stop)...")
        try:
            while True:
                print(f"SEED: {random.randint(0, 1000000)}")
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nStopped.")
    else:
        seeds = generate_seeds(args.count)
        for s in seeds:
            print(s)

if __name__ == "__main__":
    main()
