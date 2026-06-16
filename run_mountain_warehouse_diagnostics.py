from test_real_env import run_real_env_trial

for terrain_id, name in [(3, "Mountain"), (5, "Warehouse")]:
    print("=" * 60)
    print(name)
    print(run_real_env_trial(terrain_id, 1337, max_steps=3000, verbose=True))
