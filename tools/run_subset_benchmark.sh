#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

python3 constrained_adaptive_engine_bridge.py
python3 benchmark_subset.py --terrains 3 4 5 6 --trials 1 --fixed
