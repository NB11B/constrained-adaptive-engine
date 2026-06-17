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

# Package-time adapter correction: Swarm's final search vector can behave like a
# short direction cue outside warehouse maps. If treated as a literal 1m-ish
# target offset, the agent chases a moving carrot and never reaches the visible
# platform. Preserve warehouse literal behavior, but project short non-warehouse
# search vectors farther ahead before feeding CAE.
python3 - "$RUNTIME/cae_agent_impl.py" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
s = path.read_text()
needle = '''        terrain_id = self._infer_terrain(current_pos, agl, depth, dist_xy)
        self.terrain_id = terrain_id
        profile = _profile(terrain_id)

        self.engine.set_flight_params(**_landing_params(terrain_id, dist_xy, self.spawn_z, self.target_vel))
'''
replacement = '''        terrain_id = self._infer_terrain(current_pos, agl, depth, dist_xy)
        self.terrain_id = terrain_id
        profile = _profile(terrain_id)

        # Non-warehouse Swarm vectors often behave as search-direction cues.
        # Project short vectors into a useful absolute acquisition target after
        # terrain inference, while leaving warehouse's proven near-field behavior
        # unchanged.
        search_norm = float(np.linalg.norm(search_vec))
        if terrain_id != 5 and 0.05 < search_norm < 2.25 and dist_xy < 3.0:
            projected = current_pos + (search_vec / (search_norm + 1e-8)) * 9.5
            projected[2] = target_pos[2]
            target_pos = projected.astype(np.float64)
            dist_xy = float(np.linalg.norm(current_pos[:2] - target_pos[:2]))
            self.min_dist_xy = min(self.min_dist_xy, dist_xy)

        self.engine.set_flight_params(**_landing_params(terrain_id, dist_xy, self.spawn_z, self.target_vel))
'''
if needle not in s:
    raise SystemExit("Could not find terrain/flight-param block to patch")
s = s.replace(needle, replacement)
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
