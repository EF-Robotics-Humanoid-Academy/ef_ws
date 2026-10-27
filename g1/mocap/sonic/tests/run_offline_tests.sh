#!/usr/bin/env bash
# Builds and runs the offline teleop-safety unit tests (no robot, no CUDA).
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(dirname "$SCRIPT_DIR")"
OUT="$ROOT/build/test_teleop_safety"
mkdir -p "$ROOT/build"
g++ -std=c++20 -O2 -Wall -Wextra -I"$ROOT/src/g1/g1_deploy_onnx_ref/include" \
    "$SCRIPT_DIR/test_teleop_safety.cpp" -o "$OUT"
"$OUT"
