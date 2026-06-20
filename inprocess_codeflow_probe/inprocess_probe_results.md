# In-process code-flow probe

Swarm root: `/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm`
Agent class: `DroneFlightController`

## Errors

### type3_mountain

```text
Traceback (most recent call last):
  File "/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/inprocess_codeflow_probe/run_inprocess_probe.py", line 315, in <module>
    out["runs"].append(run_one(group, spec))
                       ^^^^^^^^^^^^^^^^^^^^
  File "/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/inprocess_codeflow_probe/run_inprocess_probe.py", line 240, in run_one
    env, env_meta = make_env(task)
                    ^^^^^^^^^^^^^^
  File "/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/inprocess_codeflow_probe/run_inprocess_probe.py", line 230, in make_env
    raise RuntimeError("Could not make env. Errors: " + repr(errors[:20]))
RuntimeError: Could not make env. Errors: [('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'")]

```

### type4_village

```text
Traceback (most recent call last):
  File "/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/inprocess_codeflow_probe/run_inprocess_probe.py", line 315, in <module>
    out["runs"].append(run_one(group, spec))
                       ^^^^^^^^^^^^^^^^^^^^
  File "/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/inprocess_codeflow_probe/run_inprocess_probe.py", line 240, in run_one
    env, env_meta = make_env(task)
                    ^^^^^^^^^^^^^^
  File "/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE/inprocess_codeflow_probe/run_inprocess_probe.py", line 230, in make_env
    raise RuntimeError("Could not make env. Errors: " + repr(errors[:20]))
RuntimeError: Could not make env. Errors: [('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 2 required positional arguments: 'seed' and 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('build_world', "build_world() missing 1 required positional argument: 'cli'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'"), ('make_env_with_initial_obs', "'VisualizeTarget' object has no attribute 'sim_dt'")]

```

## Runs

| Group | Seed | Steps | Terminated | Truncated | Success | Events | First correction | First corr step | First raw_search | Last correction | Last target | Last target_est |
|---|---:|---:|---|---|---|---:|---|---:|---:|---|---|---|

## Discovery

```json
{
  "swarm_root": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm",
  "agent_class": "DroneFlightController",
  "task_hits": [
    {
      "kind": "class",
      "name": "DroneModel",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/moving_drone.py"
    },
    {
      "kind": "class",
      "name": "Physics",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/moving_drone.py"
    },
    {
      "kind": "class",
      "name": "ActionType",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/moving_drone.py"
    },
    {
      "kind": "class",
      "name": "ObservationType",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/moving_drone.py"
    },
    {
      "kind": "class",
      "name": "ImageType",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/moving_drone.py"
    },
    {
      "kind": "function",
      "name": "build_world",
      "signature": "(seed: 'int', cli: 'int', *, start: 'shared.Optional[shared.Tuple[float, float, float]]' = None, goal: 'shared.Optional[shared.Tuple[float, float, float]]' = None, challenge_type: 'int' = 1, moving_platform: 'bool' = False) -> 'shared.Tuple[shared.List[int], shared.List[int], shared.Optional[float], shared.Optional[float], shared.Optional[shared.Tuple[float, float, float]], shared.Optional[shared.Tuple[float, float, float]]]'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/moving_drone.py"
    },
    {
      "kind": "class",
      "name": "MapTask",
      "signature": "(map_seed: 'int', start: 'Tuple[float, float, float]', goal: 'Tuple[float, float, float]', sim_dt: 'float', horizon: 'float', challenge_type: 'int', search_radius: 'float' = 10.0, moving_platform: 'bool' = False, version: 'str' = '1') -> None",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/protocol.py"
    },
    {
      "kind": "class",
      "name": "MapTask",
      "signature": "(map_seed: 'int', start: 'Tuple[float, float, float]', goal: 'Tuple[float, float, float]', sim_dt: 'float', horizon: 'float', challenge_type: 'int', search_radius: 'float' = 10.0, moving_platform: 'bool' = False, version: 'str' = '1') -> None",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "_resolve_params",
      "signature": "(seed: 'int', challenge_type: 'int') -> 'dict'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "_build_task_with_params",
      "signature": "(sim_dt: 'float', seed: 'int', *, challenge_type: 'int', params: 'dict', moving_platform: 'bool') -> 'MapTask'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "_build_task_for_type",
      "signature": "(sim_dt: 'float', seed: 'int', *, challenge_type: 'int', moving_platform: 'Optional[bool]') -> 'MapTask'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "get_platform_height_for_seed",
      "signature": "(seed: 'int', challenge_type: 'int' = 1) -> 'float'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "_random_start",
      "signature": "(seed_rng: 'random.Random', params: 'dict', challenge_type: 'int' = 1, seed: 'int' = 0) -> 'Tuple[float, float, float]'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "_goal_from_start",
      "signature": "(seed_rng: 'random.Random', start: 'Tuple[float, float, float]', params: 'dict', challenge_type: 'int' = 1, seed: 'int' = 0) -> 'Tuple[float, float, float]'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "random_task",
      "signature": "(sim_dt: 'float', seed: 'Optional[int]' = None) -> 'MapTask'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "task_for_seed_and_type",
      "signature": "(sim_dt: 'float', *, seed: 'int', challenge_type: 'int', moving_platform: 'Optional[bool]' = None) -> 'MapTask'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "screening_task",
      "signature": "(sim_dt: 'float', seed: 'int', *, challenge_type: 'int', distance_range: 'Tuple[float, float]', goal_height_range: 'Optional[Tuple[float, float]]', moving_platform: 'bool') -> 'MapTask'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/task_gen.py"
    },
    {
      "kind": "function",
      "name": "_build_static_world",
      "signature": "(seed: 'int', cli: 'int', *, start: 'shared.Optional[shared.Tuple[float, float, float]]', goal: 'shared.Optional[shared.Tuple[float, float, float]]', challenge_type: 'int') -> 'None'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/env_builder/build.py"
    },
    {
      "kind": "function",
      "name": "build_world",
      "signature": "(seed: 'int', cli: 'int', *, start: 'shared.Optional[shared.Tuple[float, float, float]]' = None, goal: 'shared.Optional[shared.Tuple[float, float, float]]' = None, challenge_type: 'int' = 1, moving_platform: 'bool' = False) -> 'shared.Tuple[shared.List[int], shared.List[int], shared.Optional[float], shared.Optional[float], shared.Optional[shared.Tuple[float, float, float]], shared.Optional[shared.Tuple[float, float, float]]]'",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/core/env_builder/build.py"
    },
    {
      "kind": "class",
      "name": "ObservationType",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/utils/env_factory.py"
    },
    {
      "kind": "class",
      "name": "ActionType",
      "signature": "(value, names=None, *, module=None, qualname=None, type=None, start=1, boundary=None)",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/utils/env_factory.py"
    },
    {
      "kind": "class",
      "name": "MapTask",
      "signature": "(map_seed: 'int', start: 'Tuple[float, float, float]', goal: 'Tuple[float, float, float]', sim_dt: 'float', horizon: 'float', challenge_type: 'int', search_radius: 'float' = 10.0, moving_platform: 'bool' = False, version: 'str' = '1') -> None",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/utils/env_factory.py"
    },
    {
      "kind": "class",
      "name": "_BatchHelpers",
      "signature": "(phase: Callable[[str], NoneType], on_seed_complete_guarded: Callable, build_failure_seed_meta: Callable, notify_all_failed: Callable, run_docker_cmd_quiet: Callable, cleanup_tmpdir_quiet: Callable) -> None",
      "path": "/mnt/c/Users/nateb/OneDrive/Documents/swarm_live/swarm/validator/docker/docker_evaluator_parts/batch.py"
    },
    {
      "kind": "class",
      "name": "_BatchContext",
      "signature": "(
```
