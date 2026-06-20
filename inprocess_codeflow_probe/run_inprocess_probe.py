import importlib
import inspect
import json
import os
import pkgutil
import sys
import traceback
from pathlib import Path

import numpy as np

PROBE_DIR = Path(os.environ["PROBE_DIR"])
RUNTIME_DIR = Path(os.environ["RUNTIME_DIR"])
OUT_JSON = PROBE_DIR / "inprocess_probe_events.json"
OUT_MD = PROBE_DIR / "inprocess_probe_results.md"

sys.path.insert(0, str(RUNTIME_DIR))

# Import candidate agent runtime directly.
import cae_agent_impl

# Class name may vary; find likely agent class with act().
agent_classes = []
for name, obj in vars(cae_agent_impl).items():
    if inspect.isclass(obj) and hasattr(obj, "act"):
        agent_classes.append((name, obj))

if not agent_classes:
    raise SystemExit("No agent class with act() found in cae_agent_impl.py")

AgentClass = agent_classes[0][1]

# Task oracle from prior audited generator output.
TASKS = {
    "type3_mountain": {
        "seed": 604195,
        "start": (-75.1614647121664, 65.80954720095166, -0.6810095881042477),
        "goal": (-43.92974348482822, 40.524078101749765, -1.1598125701485529),
        "challenge_type": 3,
        "search_radius": 2.241813371180955,
        "moving_platform": False,
    },
    "type4_village": {
        "seed": 604197,
        "start": (-36.23019071901968, -11.532927954433294, 0.121),
        "goal": (-39.38617181760077, 34.241410299946224, 0.0),
        "challenge_type": 4,
        "search_radius": 0.4806761143480187,
        "moving_platform": True,
    },
}

def discover_swarm():
    import swarm
    root = Path(inspect.getfile(swarm)).resolve().parent
    candidates = []
    for p in root.rglob("*.py"):
        try:
            txt = p.read_text(errors="ignore")
        except Exception:
            continue
        score = 0
        for token in ["build_world", "MovingDrone", "challenge_type", "search_radius", "env.step", "Task"]:
            if token in txt:
                score += 1
        if score:
            candidates.append((score, p, txt))
    candidates.sort(reverse=True, key=lambda x: x[0])
    return root, candidates

root, candidates = discover_swarm()

def import_by_path(root, path):
    rel = path.relative_to(root).with_suffix("")
    modname = "swarm." + ".".join(rel.parts)
    return importlib.import_module(modname)

def find_task_class_or_factory():
    task_hits = []
    for score, p, txt in candidates[:80]:
        try:
            mod = import_by_path(root, p)
        except Exception:
            continue
        for name, obj in vars(mod).items():
            if inspect.isclass(obj):
                sig = ""
                try:
                    sig = str(inspect.signature(obj))
                except Exception:
                    pass
                if any(k in sig for k in ["challenge_type", "search_radius", "moving_platform", "start", "goal", "seed"]):
                    task_hits.append(("class", name, obj, sig, p))
            elif inspect.isfunction(obj):
                sig = ""
                try:
                    sig = str(inspect.signature(obj))
                except Exception:
                    pass
                if "seed" in sig and ("challenge" in sig or "task" in name.lower() or "map" in sig):
                    task_hits.append(("function", name, obj, sig, p))
    return task_hits

def find_env_factory():
    hits = []
    for score, p, txt in candidates[:100]:
        try:
            mod = import_by_path(root, p)
        except Exception:
            continue
        for name, obj in vars(mod).items():
            if inspect.isfunction(obj):
                try:
                    sig = str(inspect.signature(obj))
                except Exception:
                    sig = ""
                source = ""
                try:
                    source = inspect.getsource(obj)
                except Exception:
                    pass
                if (
                    "build_world" in name
                    or "make_env" in name
                    or "create_env" in name
                    or "MovingDrone" in source
                    or "env" in name.lower() and "task" in sig
                ):
                    hits.append((name, obj, sig, p))
    return hits

task_hits = find_task_class_or_factory()
env_hits = find_env_factory()

discovery = {
    "swarm_root": str(root),
    "agent_class": AgentClass.__name__,
    "task_hits": [
        {"kind": k, "name": n, "signature": sig, "path": str(p)}
        for k, n, obj, sig, p in task_hits[:30]
    ],
    "env_hits": [
        {"name": n, "signature": sig, "path": str(p)}
        for n, obj, sig, p in env_hits[:30]
    ],
}

# Try to construct tasks and envs adaptively.
def instantiate_task(task_spec):
    # Prefer class with matching kwargs.
    for kind, name, obj, sig, p in task_hits:
        if kind != "class":
            continue
        kwargs = {}
        try:
            params = inspect.signature(obj).parameters
        except Exception:
            continue

        for key in params:
            if key in task_spec:
                kwargs[key] = task_spec[key]
            elif key == "start_pos":
                kwargs[key] = task_spec["start"]
            elif key == "goal_pos":
                kwargs[key] = task_spec["goal"]
            elif key == "start":
                kwargs[key] = task_spec["start"]
            elif key == "goal":
                kwargs[key] = task_spec["goal"]

        try:
            return obj(**kwargs), {"task_ctor": name, "task_path": str(p), "kwargs": kwargs}
        except Exception:
            pass

    # Try factory functions.
    for kind, name, obj, sig, p in task_hits:
        if kind != "function":
            continue
        try:
            params = inspect.signature(obj).parameters
        except Exception:
            continue
        kwargs = {}
        for key in params:
            if key in task_spec:
                kwargs[key] = task_spec[key]
            elif key in ("map_type", "group", "challenge"):
                kwargs[key] = task_spec["challenge_type"]
        try:
            return obj(**kwargs), {"task_ctor": name, "task_path": str(p), "kwargs": kwargs}
        except Exception:
            pass

    raise RuntimeError("Could not instantiate task from discovered APIs")

def make_env(task):
    errors = []
    for name, obj, sig, p in env_hits:
        try:
            params = inspect.signature(obj).parameters
        except Exception:
            params = {}

        attempts = [
            {"task": task},
            {"task": task, "render": False},
            {"task": task, "gui": False},
            {"task": task, "seed": getattr(task, "seed", None)},
        ]

        # Positional attempt too.
        for kwargs in attempts:
            kwargs = {k: v for k, v in kwargs.items() if k in params and v is not None}
            try:
                env = obj(**kwargs)
                if hasattr(env, "reset") and hasattr(env, "step"):
                    return env, {"env_factory": name, "env_path": str(p), "kwargs": kwargs}
            except Exception as e:
                errors.append((name, str(e)[:180]))

        try:
            env = obj(task)
            if hasattr(env, "reset") and hasattr(env, "step"):
                return env, {"env_factory": name, "env_path": str(p), "args": ["task"]}
        except Exception as e:
            errors.append((name, str(e)[:180]))

    raise RuntimeError("Could not make env. Errors: " + repr(errors[:20]))

def normalize_obs(obs):
    # Benchmark may give tuple reset and step.
    if isinstance(obs, tuple) and len(obs) >= 1:
        return obs[0]
    return obs

def run_one(group, spec, max_steps=3200):
    task, task_meta = instantiate_task(spec)
    env, env_meta = make_env(task)

    agent = AgentClass()
    reset_out = env.reset()
    obs = normalize_obs(reset_out)

    events = []
    final = {
        "terminated": False,
        "truncated": False,
        "info": {},
        "steps": 0,
    }

    for step in range(max_steps):
        action = agent.act(obs)
        step_out = env.step(np.asarray(action)[None, :])

        if len(step_out) == 5:
            obs, reward, terminated, truncated, info = step_out
        elif len(step_out) == 4:
            obs, reward, done, info = step_out
            terminated, truncated = bool(done), False
        else:
            raise RuntimeError(f"Unexpected env.step return len={len(step_out)}")

        obs = normalize_obs(obs)

        # Harvest internal agent telemetry.
        evs = getattr(agent, "_kg_events", [])
        while len(events) < len(evs):
            e = dict(evs[len(events)])
            e["_group"] = group
            e["_seed"] = spec["seed"]
            events.append(e)

        final = {
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "info": info,
            "steps": step + 1,
            "last_action": np.asarray(action, dtype=float).round(4).tolist(),
        }

        if terminated or truncated:
            break

    try:
        env.close()
    except Exception:
        pass

    return {
        "group": group,
        "seed": spec["seed"],
        "task_meta": task_meta,
        "env_meta": env_meta,
        "final": final,
        "events": events,
        "agent_state": {
            "step_count": getattr(agent, "step_count", None),
            "terrain_id": getattr(agent, "terrain_id", None),
            "initial_dist_xy": getattr(agent, "initial_dist_xy", None),
            "target_est": None if getattr(agent, "target_est", None) is None else np.asarray(agent.target_est, dtype=float).round(4).tolist(),
        },
    }

out = {
    "discovery": discovery,
    "runs": [],
    "errors": [],
}

for group, spec in TASKS.items():
    try:
        out["runs"].append(run_one(group, spec))
    except Exception as e:
        out["errors"].append({
            "group": group,
            "error": repr(e),
            "traceback": traceback.format_exc(),
        })

OUT_JSON.write_text(json.dumps(out, indent=2, default=str))

# Markdown summary.
md = []
md.append("# In-process code-flow probe")
md.append("")
md.append(f"Swarm root: `{root}`")
md.append(f"Agent class: `{AgentClass.__name__}`")
md.append("")
if out["errors"]:
    md.append("## Errors")
    md.append("")
    for err in out["errors"]:
        md.append(f"### {err['group']}")
        md.append("")
        md.append("```text")
        md.append(err["traceback"][-4000:])
        md.append("```")
        md.append("")

md.append("## Runs")
md.append("")
md.append("| Group | Seed | Steps | Terminated | Truncated | Success | Events | First correction | First corr step | First raw_search | Last correction | Last target | Last target_est |")
md.append("|---|---:|---:|---|---|---|---:|---|---:|---:|---|---|---|")

for run in out["runs"]:
    events = run["events"]
    nonnull = [e for e in events if e.get("goal_corr") not in (None, "None")]
    first = nonnull[0] if nonnull else None
    last = nonnull[-1] if nonnull else (events[-1] if events else None)
    info = run["final"].get("info") or {}
    md.append(
        f"| {run['group']} | {run['seed']} | {run['final']['steps']} | "
        f"{run['final']['terminated']} | {run['final']['truncated']} | {info.get('success')} | "
        f"{len(events)} | {None if first is None else first.get('goal_corr')} | "
        f"{None if first is None else first.get('step')} | "
        f"{None if first is None else first.get('raw_search_dist')} | "
        f"{None if last is None else last.get('goal_corr')} | "
        f"`{None if last is None else last.get('target')}` | "
        f"`{run['agent_state'].get('target_est')}` |"
    )

md.append("")
md.append("## Discovery")
md.append("")
md.append("```json")
md.append(json.dumps(discovery, indent=2)[:8000])
md.append("```")
OUT_MD.write_text("\n".join(md) + "\n")

print(OUT_MD.read_text())
