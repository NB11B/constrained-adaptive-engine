# Local Swarm / WSL Setup Notes

These notes document the local WSL setup used to run CAE against a copied Crazyflow/Swarm simulator package.

## Known Environment Split

The local `crazyflow_swarm_sim` checkout may contain the `swarm` package only inside a Windows virtual environment:

```text
crazyflow_swarm_sim/.venv/Lib/site-packages/swarm
```

Do not add the full Windows virtualenv site-packages path to `PYTHONPATH` or `SWARM_REPO` from WSL. That will mix Windows wheels with Linux Python and can produce errors such as:

```text
os.add_dll_directory is unavailable
numpy.core.multiarray failed to import
```

## Recommended WSL Virtualenv

From the CAE repo:

```bash
python3 -m venv .venv-wsl
source .venv-wsl/bin/activate
pip install --upgrade pip setuptools wheel
```

Install Linux-native dependencies:

```bash
pip install "numpy==1.26.4" pybullet gymnasium scipy matplotlib opencv-python pillow tqdm msgpack pydantic requests loguru rich gym-pybullet-drones
```

NumPy is pinned below 2.x because some PyBullet builds are compiled against NumPy 1.x.

## Copy Pure-Python Swarm Package

If the only available `swarm` package is inside the Windows venv, copy only the `swarm/` package into the WSL venv:

```bash
cd /mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine
source .venv-wsl/bin/activate

WSL_SITE=$(python3 - <<'PY'
import site
print(site.getsitepackages()[0])
PY
)

rm -rf "$WSL_SITE/swarm"
cp -r /mnt/c/Users/nateb/OneDrive/Documents/crazyflow_swarm_sim/.venv/Lib/site-packages/swarm "$WSL_SITE/swarm"
```

Then make sure no Windows virtualenv path is active:

```bash
unset SWARM_REPO
```

## Import Test

```bash
python3 - <<'PY'
from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env
import numpy, pybullet
print("Swarm import OK")
print("numpy", numpy.__version__)
print("pybullet OK")
PY
```

## Action-Space Check

Some local Swarm builds expose a 4-wide action space while validator-style builds may expose a 5-wide action space.

Run:

```bash
python3 - <<'PY'
from swarm.validator.task_gen import task_for_seed_and_type
from swarm.utils.env_factory import make_env

task = task_for_seed_and_type(sim_dt=1/50, seed=1337, challenge_type=3)
env = make_env(task, gui=False)
obs, _ = env.reset()

print("action_space:", env.action_space)
print("action_space.shape:", getattr(env.action_space, "shape", None))
print("state shape:", obs["state"].shape)
print("depth shape:", obs["depth"].shape)

env.close()
PY
```

Observed local WSL result:

```text
action_space: Box(-1.0, 1.0, (1, 4), float32)
action_space.shape: (1, 4)
state shape: (116,)
depth shape: (128, 128, 1)
```

The CAE harness should adapt at the `env.step()` boundary if the local environment expects 4 commands instead of 5.

## Build CAE Native Library

```bash
gcc -O3 -shared -fPIC -Iinclude \
  -o libadaptive_controller.so \
  src/adaptation_controller.c \
  src/psmsl_depth_processor.c \
  -lm
```

## Run Validation

```bash
python3 constrained_adaptive_engine_bridge.py
python3 test_real_env.py
python3 benchmark_subset.py --terrains 3 4 5 6 --trials 1 --fixed
```

## Common Fixes

If PyBullet fails with a NumPy ABI message, reinstall NumPy 1.x:

```bash
pip install --force-reinstall "numpy==1.26.4" pybullet
```

If a missing package appears, install it into `.venv-wsl`, not the Windows venv.
