#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build/swarm_cae_agent}"
RUNTIME="$OUT/cae_runtime"

rm -rf "$OUT"
mkdir -p "$OUT" "$RUNTIME/src" "$RUNTIME/include"

# Swarm-packaged entrypoint: small loader only.
cp "$ROOT/swarm_submission/cae_agent/drone_agent_wrapper.py" "$OUT/drone_agent.py"
cp "$ROOT/swarm_submission/cae_agent/requirements.txt" "$OUT/requirements.txt"

# Real CAE implementation goes inside cae_runtime.zip.
cp "$ROOT/swarm_submission/cae_agent/drone_agent.py" "$RUNTIME/cae_agent_impl.py"

# Package-time adapter correction and optional flight recorder for Swarm.
# The recorder is disabled by default and only activates when
# CAE_FLIGHT_RECORDER=1 is present in the model process environment.
python3 - "$RUNTIME/cae_agent_impl.py" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
s = path.read_text()

# Optional JSONL flight recorder for adapter-transition debugging.
if "import json" not in s:
    s = s.replace("import math\n", "import math\nimport json\nimport os\n")

if "CAE_FLIGHT_RECORDER" not in s:
    s = s.replace(
        "DEFAULT_PROFILE_ID = 2\n",
        '''DEFAULT_PROFILE_ID = 2
CAE_FLIGHT_RECORDER = os.environ.get("CAE_FLIGHT_RECORDER", "0") == "1"
CAE_FLIGHT_RECORDER_PATH = os.environ.get("CAE_FLIGHT_RECORDER_PATH", "/tmp/cae_flight_recorder.jsonl")


def _record_flight_transition(record: Dict[str, object]) -> None:
    if not CAE_FLIGHT_RECORDER:
        return
    try:
        with open(CAE_FLIGHT_RECORDER_PATH, "a", encoding="utf-8") as f:
            f.write(json.dumps(record, separators=(",", ":")) + "\\n")
    except Exception:
        pass

'''
    )

old_landing = '''def _landing_params(terrain_id: int, dist_xy: float, spawn_z: float, target_vel: np.ndarray) -> Dict[str, object]:
    profile = _profile(terrain_id)
    landing_committed = dist_xy < 4.25

    if landing_committed:
        cruise_altitude = 0.50
        safety_radius = 0.52 if terrain_id == 5 else 0.62
        mode_v = 36.0 if terrain_id == 5 else 42.0
        attraction_gain = 16.0
    else:
        cruise_altitude = spawn_z + float(profile["transit_alt"])
        safety_radius = float(profile["safety"])
        mode_v = float(profile["mode_v"])
        attraction_gain = 13.5
'''
new_landing = '''def _landing_params(terrain_id: int, dist_xy: float, spawn_z: float, target_vel: np.ndarray) -> Dict[str, object]:
    profile = _profile(terrain_id)
    landing_committed = dist_xy < 4.25

    if landing_committed:
        cruise_altitude = 0.50
        safety_radius = 0.52 if terrain_id == 5 else 0.62
        mode_v = 36.0 if terrain_id == 5 else 42.0
        attraction_gain = 16.0
    else:
        if terrain_id == 3:
            cruise_altitude = spawn_z + float(profile["transit_alt"])
        elif terrain_id == 5:
            cruise_altitude = spawn_z + float(profile["transit_alt"])
        else:
            # Non-warehouse/non-mountain maps should acquire laterally at low
            # altitude. The previous spawn+transit profile produced persistent
            # upward commands and missed visible platforms.
            cruise_altitude = min(spawn_z + float(profile["transit_alt"]), 1.25)
        safety_radius = float(profile["safety"])
        mode_v = float(profile["mode_v"])
        attraction_gain = 13.5
'''
if old_landing in s:
    s = s.replace(old_landing, new_landing)

old_assist = '''            elif terrain_id == 3 and dist_xy > 5.0:
                target_alt_assist = target_pos[2] + 6.3
            else:
                target_alt_assist = target_pos[2] + 1.4
'''
new_assist = '''            elif terrain_id == 3 and dist_xy > 5.0:
                target_alt_assist = target_pos[2] + 6.3
            elif terrain_id == 5:
                target_alt_assist = target_pos[2] + 1.4
            else:
                # Hold a low acquisition shelf outside warehouse/mountain so
                # the adapter does not climb over the target during approach.
                target_alt_assist = target_pos[2] + 0.75
'''
if old_assist in s:
    s = s.replace(old_assist, new_assist)

record_needle = '''        if mountain_descent_corridor:
            self.descent_corridor_ticks += 1

        self.step_count += 1
'''
record_inject = '''        if CAE_FLIGHT_RECORDER and (self.step_count < 80 or self.step_count % 100 == 0):
            close_fraction, mid_fraction, mean_depth = _depth_metrics(depth)
            _record_flight_transition({
                "step": int(self.step_count),
                "terrain_id": int(terrain_id),
                "pos": [float(x) for x in current_pos.tolist()],
                "vel": [float(x) for x in current_vel.tolist()],
                "rpy": [float(x) for x in current_rpy.tolist()],
                "agl": float(agl),
                "search_vec": [float(x) for x in search_vec.tolist()],
                "search_norm": float(np.linalg.norm(search_vec)),
                "target_pos": [float(x) for x in target_pos.tolist()],
                "target_vel": [float(x) for x in self.target_vel.tolist()],
                "dist_xy": float(dist_xy),
                "min_dist_xy": float(self.min_dist_xy),
                "depth_close_fraction": float(close_fraction),
                "depth_mid_fraction": float(mid_fraction),
                "depth_mean": float(mean_depth),
                "landing_phase": bool(cs.landing_phase),
                "descent_phase": bool(cs.descent_phase),
                "collision_risk": float(cs.collision_risk_score),
                "c_output": [float(vx), float(vy), float(vz), float(total_speed), float(yaw_cmd)],
                "final_action": [float(x) for x in action.tolist()],
                "terminal_ticks": int(self.terminal_ticks),
                "settle_ticks": int(self.settle_ticks),
                "descent_corridor_ticks": int(self.descent_corridor_ticks),
            })

        if mountain_descent_corridor:
            self.descent_corridor_ticks += 1

        self.step_count += 1
'''
if "_record_flight_transition" in s and "depth_close_fraction" not in s:
    if record_needle not in s:
        raise SystemExit("Could not find recorder injection point")
    s = s.replace(record_needle, record_inject)
elif "depth_close_fraction" not in s:
    if record_needle not in s:
        raise SystemExit("Could not find recorder injection point")
    s = s.replace(record_needle, record_inject)

path.write_text(s)
PY

cp "$ROOT/constrained_adaptive_engine_bridge.py" "$RUNTIME/constrained_adaptive_engine_bridge.py"
cp "$ROOT/src/adaptation_controller.c" "$RUNTIME/src/adaptation_controller.c"
cp "$ROOT/src/psmsl_depth_processor.c" "$RUNTIME/src/psmsl_depth_processor.c"
cp "$ROOT/include/"*.h "$RUNTIME/include/"

# Include prebuilt Linux shared object when available. If absent, the bridge can
# compile from src/include when gcc/clang is available in the validator image.
if [[ -f "$ROOT/libadaptive_controller.so" ]]; then
  cp "$ROOT/libadaptive_controller.so" "$RUNTIME/libadaptive_controller.so"
fi

(
  cd "$RUNTIME"
  python3 - <<'PY'
from pathlib import Path
import zipfile

out = Path("../cae_runtime.zip")
if out.exists():
    out.unlink()

with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as zf:
    for path in sorted(Path(".").rglob("*")):
        if path.is_file():
            zf.write(path, path.as_posix())
PY
)

rm -rf "$RUNTIME"

echo "Built Swarm CAE agent source at: $OUT"
find "$OUT" -maxdepth 2 -type f | sort
echo
echo "Expected Swarm package files:"
echo "  drone_agent.py"
echo "  requirements.txt"
echo "  cae_runtime.zip"
