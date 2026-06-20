import json
import sys
import inspect
import traceback
from pathlib import Path
from collections import Counter, defaultdict

import numpy as np

from swarm.protocol import MapTask
from swarm.utils.env_factory import make_env_with_initial_obs
from swarm.validator.reward import flight_reward

PROBE_DIR = Path(__file__).resolve().parent
RUNTIME_DIR = PROBE_DIR / "runtime"
OUT_JSON = PROBE_DIR / "inprocess_probe_events.json"
OUT_MD = PROBE_DIR / "inprocess_probe_results.md"

sys.path.insert(0, str(RUNTIME_DIR))
import cae_agent_impl

# Find the actual controller class.
agent_classes = []
for name, obj in vars(cae_agent_impl).items():
    if inspect.isclass(obj) and hasattr(obj, "act"):
        agent_classes.append((name, obj))

if not agent_classes:
    raise SystemExit("No class with act() found in cae_agent_impl.py")

AgentClass = dict(agent_classes).get("DroneFlightController", agent_classes[0][1])

TASKS = {
    "type3_mountain": dict(
        map_seed=604195,
        start=(-75.1614647121664, 65.80954720095166, -0.6810095881042477),
        goal=(-43.92974348482822, 40.524078101749765, -1.1598125701485529),
        sim_dt=0.02,
        horizon=60,
        challenge_type=3,
        search_radius=2.241813371180955,
        moving_platform=False,
        version="1",
    ),
    "type4_village": dict(
        map_seed=604197,
        start=(-36.23019071901968, -11.532927954433294, 0.121),
        goal=(-39.38617181760077, 34.241410299946224, 0.0),
        sim_dt=0.02,
        horizon=60,
        challenge_type=4,
        search_radius=0.4806761143480187,
        moving_platform=True,
        version="1",
    ),
}

def unwrap_reset(x):
    if isinstance(x, tuple) and len(x) >= 1:
        return x[0]
    return x

def run_one(group, kwargs):
    task = MapTask(**kwargs)
    env, obs = make_env_with_initial_obs(task, gui=False)
    agent = AgentClass()

    events = []
    final = {
        "terminated": False,
        "truncated": False,
        "info": {},
        "steps": 0,
        "score": None,
    }

    try:
        obs = unwrap_reset(obs)

        # Reset agent state if it exposes reset().
        if hasattr(agent, "reset"):
            try:
                agent.reset()
            except Exception:
                pass

        max_steps = int(task.horizon / task.sim_dt) + 50

        for step in range(max_steps):
            action = agent.act(obs)
            action_arr = np.asarray(action, dtype=np.float32)
            if action_arr.ndim == 1:
                action_for_env = action_arr[None, :]
            else:
                action_for_env = action_arr

            step_out = env.step(action_for_env)

            if len(step_out) == 5:
                obs, _r, terminated, truncated, info = step_out
            elif len(step_out) == 4:
                obs, _r, done, info = step_out
                terminated, truncated = bool(done), False
            else:
                raise RuntimeError(f"Unexpected step_out len={len(step_out)}")

            obs = unwrap_reset(obs)

            evs = getattr(agent, "_kg_events", [])
            while len(events) < len(evs):
                e = dict(evs[len(events)])
                e["_group"] = group
                e["_seed"] = task.map_seed
                events.append(e)

            final = {
                "terminated": bool(terminated),
                "truncated": bool(truncated),
                "info": dict(info or {}),
                "steps": step + 1,
                "last_action": np.asarray(action_arr, dtype=float).round(4).tolist(),
                "score": None,
            }

            if terminated or truncated:
                break

        t_sim = float(final["steps"]) * float(task.sim_dt)
        final["score"] = float(
            flight_reward(
                success=bool(final["info"].get("success", False)),
                t=t_sim,
                horizon=task.horizon,
                task=task,
                min_clearance=final["info"].get("min_clearance"),
                collision=bool(final["info"].get("collision", False)),
                legitimate_model=True,
            )
        )

    finally:
        try:
            env.close()
        except Exception:
            pass

    return {
        "group": group,
        "task": kwargs,
        "final": final,
        "events": events,
        "agent_state": {
            "step_count": getattr(agent, "step_count", None),
            "terrain_id": getattr(agent, "terrain_id", None),
            "initial_dist_xy": None if getattr(agent, "initial_dist_xy", None) is None else float(agent.initial_dist_xy),
            "target_est": None if getattr(agent, "target_est", None) is None else np.asarray(agent.target_est, dtype=float).round(4).tolist(),
            "min_dist_xy": None if getattr(agent, "min_dist_xy", None) is None else float(agent.min_dist_xy),
        },
    }

out = {
    "agent_class": AgentClass.__name__,
    "runs": [],
    "errors": [],
}

for group, kwargs in TASKS.items():
    try:
        out["runs"].append(run_one(group, kwargs))
    except Exception:
        out["errors"].append({
            "group": group,
            "traceback": traceback.format_exc(),
        })

OUT_JSON.write_text(json.dumps(out, indent=2, default=str))

md = []
md.append("# In-process code-flow probe v2")
md.append("")
md.append(f"Agent class: `{AgentClass.__name__}`")
md.append("")
if out["errors"]:
    md.append("## Errors")
    md.append("")
    for err in out["errors"]:
        md.append(f"### {err['group']}")
        md.append("")
        md.append("```text")
        md.append(err["traceback"][-5000:])
        md.append("```")
        md.append("")

md.append("## Runs")
md.append("")
md.append("| Group | Steps | Terminated | Truncated | Success | Score | Events | First correction | First corr step | First init_dist | First raw_search | Last correction | Last target | Last target_est | Agent terrain_id | Agent min_dist_xy |")
md.append("|---|---:|---|---|---|---:|---:|---|---:|---:|---:|---|---|---|---:|---:|")

for run in out["runs"]:
    events = run["events"]
    nonnull = [e for e in events if e.get("goal_corr") not in (None, "None")]
    first = nonnull[0] if nonnull else None
    last = nonnull[-1] if nonnull else (events[-1] if events else None)
    final = run["final"]
    info = final.get("info") or {}
    st = run["agent_state"]

    md.append(
        f"| {run['group']} | {final.get('steps')} | {final.get('terminated')} | "
        f"{final.get('truncated')} | {info.get('success')} | {final.get('score')} | "
        f"{len(events)} | {None if first is None else first.get('goal_corr')} | "
        f"{None if first is None else first.get('step')} | "
        f"{None if first is None else first.get('init_dist')} | "
        f"{None if first is None else first.get('raw_search_dist')} | "
        f"{None if last is None else last.get('goal_corr')} | "
        f"`{None if last is None else last.get('target')}` | "
        f"`{st.get('target_est')}` | "
        f"{st.get('terrain_id')} | {st.get('min_dist_xy')} |"
    )

md.append("")
md.append("## Correction event counts")
md.append("")
for run in out["runs"]:
    counts = Counter(str(e.get("goal_corr")) for e in run["events"])
    md.append(f"- `{run['group']}`: `{dict(counts)}`")
md.append("")
md.append("## First 20 correction events per run")
md.append("")
for run in out["runs"]:
    md.append(f"### {run['group']}")
    md.append("")
    corr_events = [e for e in run["events"] if e.get("goal_corr") not in (None, "None")]
    md.append("```json")
    md.append(json.dumps(corr_events[:20], indent=2))
    md.append("```")
    md.append("")

OUT_MD.write_text("\n".join(md) + "\n")
print(OUT_MD.read_text())
