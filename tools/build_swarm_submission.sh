#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$ROOT/build/swarm_cae_agent}"

rm -rf "$OUT"
mkdir -p "$OUT/src" "$OUT/include"

cp "$ROOT/swarm_submission/cae_agent/drone_agent.py" "$OUT/drone_agent.py"
cp "$ROOT/swarm_submission/cae_agent/requirements.txt" "$OUT/requirements.txt"
cp "$ROOT/constrained_adaptive_engine_bridge.py" "$OUT/constrained_adaptive_engine_bridge.py"
cp "$ROOT/src/adaptation_controller.c" "$OUT/src/adaptation_controller.c"
cp "$ROOT/src/psmsl_depth_processor.c" "$OUT/src/psmsl_depth_processor.c"
cp "$ROOT/include/"*.h "$OUT/include/"

# Include a prebuilt Linux shared object when available. The bridge can also
# compile from src/include when the evaluation container has gcc/clang.
if [[ -f "$ROOT/libadaptive_controller.so" ]]; then
  cp "$ROOT/libadaptive_controller.so" "$OUT/libadaptive_controller.so"
fi

cat > "$OUT/README_CAE_SUBMISSION.txt" <<'EOF'
Constrained Adaptive Engine Swarm submission source folder.

Required Swarm files:
- drone_agent.py
- requirements.txt

CAE runtime files:
- constrained_adaptive_engine_bridge.py
- src/adaptation_controller.c
- src/psmsl_depth_processor.c
- include/*.h
- libadaptive_controller.so when prebuilt locally

Package with Swarm CLI:
  swarm model test --source build/swarm_cae_agent/
  swarm model package --source build/swarm_cae_agent/
  swarm model verify --model Submission/submission.zip
EOF

echo "Built Swarm CAE agent source at: $OUT"
find "$OUT" -maxdepth 2 -type f | sort
