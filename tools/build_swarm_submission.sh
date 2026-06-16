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
