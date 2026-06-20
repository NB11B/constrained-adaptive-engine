import inspect
import json
import shutil
import sys
import traceback
import zipfile
from pathlib import Path

import numpy as np

from swarm.protocol import MapTask
from swarm.utils.env_factory import make_env_with_initial_obs
from swarm.validator.reward import flight_reward

CANDIDATE_DIR = Path("/mnt/c/Users/nateb/OneDrive/Documents/constrained-adaptive-engine_NEXT_CANDIDATE")
AUDIT_DIR = CANDIDATE_DIR / "mountain_failure_edge_audit"
JSON_OUT = AUDIT_DIR / "mountain_failure_edge.json"
REPORT = AUDIT_DIR / "mountain_failure_edge_report.md"

ZIPS = {
    "v1_locked": CANDIDATE_DIR / "build/submission_standardized_goal_blocks_v1.zip",
    "terrain3_override": CANDIDATE_DIR / "build/submission_standardized_goal_blocks_v1_true_mountain_override.zip",
    "cruise_shim": CANDIDATE_DIR / "build/submission_standardized_goal_blocks_v1_true_mountain_cruise_shim.zip",
    "xy_push": CANDIDATE_DIR / "build/submission_standardized_goal_blocks_v1_true_mountain_xy_push.zip",
}

TASK = MapTask(
    map_seed=604195,
    start=(-75.1614647121664, 65.80954720095166, -0.6810095881042477),
    goal=(-43.92974348482822, 40.524078101749765, -1.1598125701485529),
    sim_dt=0.02,
    horizon=60,
    challenge_type=3,
    search_radius=2.241813371180955,
    moving_platform=False,
    version="1",
)

def patch_runtime(runtime_dir):
    impl = runtime_dir / "cae_agent_impl.py"
    s = impl.read_text()

    if "self._kg_events" not in s:
        for anchor in [
            "        self.step_count = 0\n",
            "        self.trace_limit = int(os.environ.get(\"CAE_TRACE_LIMIT\", \"0\"))\n",
        ]:
            if anchor in s:
                s = s.replace(anchor, anchor + "        self._kg_events = []\n", 1)
                break

    anchor = "        terrain_id = self._infer_terrain(current_pos, agl, depth, dist_xy)\n"
    insert = '''        # FAILURE_EDGE_TELEMETRY
        if (
            applied_goal_correction is not None
            or self.step_count < 8
            or self.step_count % 50 == 0
        ):
            try:
                self._kg_events.append(
                    {
                        "step": int(self.step_count),
                        "phase": "pre_terrain",
                        "goal_corr": applied_goal_correction,
                        "init_dist": None if self.initial_dist_xy is None else round(float(self.initial_dist_xy), 4),
                        "raw_search_dist": round(float(raw_search_dist_xy), 4),
                        "pos": np.asarray(current_pos, dtype=float).round(4).tolist(),
                        "vel": np.asarray(current_vel, dtype=float).round(4).tolist(),
                        "target": np.asarray(target_pos, dtype=float).round(4).tolist(),
                        "target_est": None if self.target_est is None else np.asarray(self.target_est, dtype=float).round(4).tolist(),
                        "search_vec": np.asarray(search_vec, dtype=float).round(4).tolist(),
                        "dist_xy": round(float(dist_xy), 4),
                        "agl": round(float(agl), 4),
                    }
                )
            except Exception:
                pass

'''
    if "FAILURE_EDGE_TELEMETRY" not in s and anchor in s:
        s = s.replace(anchor, insert + anchor, 1)

    # Also capture post-control action/collision risk if anchor exists.
    action_anchor = "        action = _action_from_velocity(vx, vy, vz, total_speed, yaw_norm)\n"
    action_insert = '''        # FAILURE_EDGE_ACTION_TELEMETRY
        try:
            if hasattr(self, "_kg_events") and self._kg_events and self._kg_events[-1].get("step") == int(self.step_count):
                self._kg_events[-1]["control_output"] = [round(float(x), 4) for x in [vx, vy, vz, total_speed, yaw_cmd]]
        except Exception:
            pass

'''
    if "FAILURE_EDGE_ACTION_TELEMETRY" not in s and action_anchor in s:
        s = s.replace(action_anchor, action_anchor + action_insert, 1)

    impl.write_text(s)

def load_agent(runtime_dir):
    sys.path.insert(0, str(runtime_dir))
    try:
        if "cae_agent_impl" in sys.modules:
            del sys.modules["cae_agent_impl"]
        import cae_agent_impl
        classes = [
            obj for name, obj in vars(cae_agent_impl).items()
            if inspect.isclass(obj) and hasattr(obj, "act")
        ]
        if not classes:
            raise RuntimeError("No class with act() found")
        cls = next((c for c in classes if c.__name__ == "DroneFlightController"), classes[0])
        return cls
    finally:
        try:
            sys.path.remove(str(runtime_dir))
        except ValueError:
            pass

def unpack_runtime(label, zip_path):
    work = AUDIT_DIR / label
    if work.exists():
        shutil.rmtree(work)
    sub = work / "submission"
    rt = work / "runtime"
    sub.mkdir(parents=True)
    rt.mkdir(parents=True)

    with zipfile.ZipFile(zip_path) as z:
        z.extractall(sub)
    with zipfile.ZipFile(sub / "cae_runtime.zip") as z:
        z.extractall(rt)
    patch_runtime(rt)
    return rt

def run_variant(label, zip_path):
    rt = unpack_runtime(label, zip_path)
    Agent = load_agent(rt)
    agent = Agent()

    env, obs = make_env_with_initial_obs(TASK, gui=False)

    events = []
    step_infos = []
    final = {}

    try:
        max_steps = int(TASK.horizon / TASK.sim_dt) + 50

        for step in range(max_steps):
            act = agent.act(obs)
            arr = np.asarray(act, dtype=np.float32)
            if arr.ndim == 1:
                arr_env = arr[None, :]
            else:
                arr_env = arr

            obs, _r, terminated, truncated, info = env.step(arr_env)

            evs = getattr(agent, "_kg_events", [])
            while len(events) < len(evs):
                e = dict(evs[len(events)])
                events.append(e)

            if step % 25 == 0 or terminated or truncated:
                step_infos.append({
                    "step": step + 1,
                    "terminated": bool(terminated),
                    "truncated": bool(truncated),
                    "info": dict(info or {}),
                    "action": arr.round(4).tolist(),
                    "last_event": events[-1] if events else None,
                })

            if terminated or truncated:
                break

        sim_t = (step + 1) * TASK.sim_dt
        score = flight_reward(
            success=bool(info.get("success", False)),
            t=sim_t,
            horizon=TASK.horizon,
            task=TASK,
            min_clearance=info.get("min_clearance"),
            collision=bool(info.get("collision", False)),
            legitimate_model=True,
        )

        final = {
            "steps": step + 1,
            "sim_time": sim_t,
            "terminated": bool(terminated),
            "truncated": bool(truncated),
            "success": bool(info.get("success", False)),
            "collision": bool(info.get("collision", False)),
            "min_clearance": info.get("min_clearance"),
            "distance_to_goal": info.get("distance_to_goal"),
            "landing_stable_time": info.get("landing_stable_time"),
            "score": float(score),
            "agent_terrain_id": getattr(agent, "terrain_id", None),
            "agent_initial_dist_xy": getattr(agent, "initial_dist_xy", None),
            "agent_min_dist_xy": getattr(agent, "min_dist_xy", None),
            "agent_target_est": None if getattr(agent, "target_est", None) is None else np.asarray(agent.target_est).round(4).tolist(),
        }

    except Exception:
        final = {
            "error": traceback.format_exc(),
        }
    finally:
        try:
            env.close()
        except Exception:
            pass

    corr_events = [e for e in events if e.get("goal_corr") not in (None, "None")]
    return {
        "label": label,
        "zip": str(zip_path),
        "final": final,
        "event_count": len(events),
        "first_corr_event": corr_events[0] if corr_events else None,
        "last_event": events[-1] if events else None,
        "last_20_events": events[-20:],
        "step_infos_tail": step_infos[-12:],
    }

out = {
    "task": {
        "seed": TASK.map_seed,
        "challenge_type": TASK.challenge_type,
        "sim_dt": TASK.sim_dt,
        "horizon": TASK.horizon,
    },
    "runs": [],
}

for label, zip_path in ZIPS.items():
    if not zip_path.exists():
        out["runs"].append({
            "label": label,
            "zip": str(zip_path),
            "missing": True,
        })
        continue
    out["runs"].append(run_variant(label, zip_path))

JSON_OUT.write_text(json.dumps(out, indent=2, default=str))

md = []
md.append("# Mountain failure-edge audit")
md.append("")
md.append("## Summary")
md.append("")
md.append("| Variant | Steps | SimT | Terminated | Truncated | Success | Collision | Min clearance | Dist goal | Score | Terrain | Min dist XY | First corr step | First corr target | Last pos | Last target |")
md.append("|---|---:|---:|---|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|")

for r in out["runs"]:
    if r.get("missing"):
        md.append(f"| {r['label']} | MISSING | | | | | | | | | | | | | | |")
        continue

    f = r["final"]
    first = r.get("first_corr_event") or {}
    last = r.get("last_event") or {}
    md.append(
        f"| {r['label']} | {f.get('steps')} | {f.get('sim_time')} | "
        f"{f.get('terminated')} | {f.get('truncated')} | {f.get('success')} | "
        f"{f.get('collision')} | {f.get('min_clearance')} | {f.get('distance_to_goal')} | "
        f"{f.get('score')} | {f.get('agent_terrain_id')} | {f.get('agent_min_dist_xy')} | "
        f"{first.get('step')} | `{first.get('target')}` | `{last.get('pos')}` | `{last.get('target')}` |"
    )

md.append("")
md.append("## Tail step info")
for r in out["runs"]:
    if r.get("missing"):
        continue
    md.append(f"\n### {r['label']}\n")
    md.append("```json")
    md.append(json.dumps(r["step_infos_tail"], indent=2, default=str))
    md.append("```")

REPORT.write_text("\n".join(md) + "\n")
print(REPORT.read_text())
