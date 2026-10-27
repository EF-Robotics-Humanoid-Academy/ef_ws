#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
BUILD_DIR="$SCRIPT_DIR/build"

require_matching_package_arch "$SCRIPT_DIR"

onnx_root="${onnxruntime_ROOT:-}"
if [[ -z "$onnx_root" && -d "$SCRIPT_DIR/thirdparty/onnxruntime" ]]; then
    onnx_root="$SCRIPT_DIR/thirdparty/onnxruntime"
elif [[ -z "$onnx_root" && -d /opt/onnxruntime ]]; then
    onnx_root=/opt/onnxruntime
fi

cmake_args=(
    -S "$SCRIPT_DIR"
    -B "$BUILD_DIR"
    -DCMAKE_BUILD_TYPE=Release
)
if [[ -n "$onnx_root" ]]; then
    cmake_args+=("-Donnxruntime_ROOT=$onnx_root")
fi
if [[ -n "${TensorRT_ROOT:-}" ]]; then
    cmake_args+=("-DTensorRT_ROOT=$TensorRT_ROOT")
fi

cmake "${cmake_args[@]}"
cmake --build "$BUILD_DIR" --target g1_deploy_onnx_ref --parallel "${CMAKE_BUILD_PARALLEL_LEVEL:-$(nproc)}"

echo "Built $SCRIPT_DIR/bin/g1_deploy_onnx_ref"
