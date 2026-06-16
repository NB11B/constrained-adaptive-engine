#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

rm -f libadaptive_controller.so libadaptive_controller.dll

gcc -O3 -shared -fPIC -Iinclude \
  -o libadaptive_controller.so \
  src/adaptation_controller.c \
  src/psmsl_depth_processor.c \
  -lm

echo "Built $ROOT_DIR/libadaptive_controller.so"
