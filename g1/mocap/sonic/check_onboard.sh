#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
errors=0
TARGET_ARCH="$(package_arch "$SCRIPT_DIR")"
HOST_ARCH="$(normalize_arch "$(uname -m)")"
EXPECTED_TRT="$(expected_tensorrt_version "$TARGET_ARCH")"

fail() {
    echo "[FAIL] $*" >&2
    errors=$((errors + 1))
}

pass() {
    echo "[ OK ] $*"
}

read_trt_macro() {
    local header="$1"
    local macro="$2"
    local value
    value="$(awk -v name="$macro" '$1 == "#define" && $2 == name {print $3; exit}' "$header")"
    if [[ "$value" =~ ^[0-9]+$ ]]; then
        echo "$value"
    else
        awk -v name="$value" '$1 == "#define" && $2 == name {print $3; exit}' "$header"
    fi
}

if [[ "$HOST_ARCH" == "$TARGET_ARCH" ]]; then
    pass "CPU architecture matches package target: $TARGET_ARCH"
else
    fail "CPU architecture is $HOST_ARCH, package target is $TARGET_ARCH"
fi

if [[ "$TARGET_ARCH" == "aarch64" ]]; then
    if [[ -f /etc/nv_tegra_release ]] || [[ -r /proc/device-tree/model ]]; then
        pass "Jetson platform detected"
    else
        fail "Jetson platform was not detected"
    fi
elif [[ -r /etc/os-release ]]; then
    distro_id="$(awk -F= '$1 == "ID" {gsub(/\"/, "", $2); print $2}' /etc/os-release)"
    if [[ "$distro_id" == "ubuntu" ]]; then
        pass "Ubuntu x86_64 platform detected"
    else
        echo "[WARN] x86_64 package is validated on Ubuntu; detected '$distro_id'"
    fi
fi

trt_header=""
for candidate in \
    "${TensorRT_ROOT:-}/include/NvInferVersion.h" \
    /usr/include/aarch64-linux-gnu/NvInferVersion.h \
    /usr/include/x86_64-linux-gnu/NvInferVersion.h \
    /usr/include/NvInferVersion.h; do
    if [[ -n "$candidate" && -f "$candidate" ]]; then
        trt_header="$candidate"
        break
    fi
done

if [[ -z "$trt_header" ]]; then
    fail "TensorRT headers were not found; set TensorRT_ROOT"
else
    trt_major="$(read_trt_macro "$trt_header" NV_TENSORRT_MAJOR)"
    trt_minor="$(read_trt_macro "$trt_header" NV_TENSORRT_MINOR)"
    if [[ "$trt_major.$trt_minor" == "$EXPECTED_TRT" ]]; then
        pass "TensorRT version is $EXPECTED_TRT"
    elif [[ "${ALLOW_UNSUPPORTED_TENSORRT:-0}" == "1" ]]; then
        echo "[WARN] TensorRT is $trt_major.$trt_minor; the validated $TARGET_ARCH version is $EXPECTED_TRT"
    else
        fail "TensorRT is $trt_major.$trt_minor, but the $TARGET_ARCH deployment requires $EXPECTED_TRT"
    fi
fi

required_files=(
    policy/release/model_decoder.onnx
    policy/release/model_encoder.onnx
    policy/release/observation_config.yaml
    policy/low_latency/model_decoder.onnx
    policy/low_latency/model_encoder.onnx
    policy/low_latency/observation_config.yaml
    reference/seed/joint_pos.csv
    "thirdparty/unitree_sdk2/lib/$TARGET_ARCH/libunitree_sdk2.a"
    "thirdparty/unitree_sdk2/thirdparty/lib/$TARGET_ARCH/libddsc.so"
    "thirdparty/unitree_sdk2/thirdparty/lib/$TARGET_ARCH/libddscxx.so"
)

for relative_path in "${required_files[@]}"; do
    path="$SCRIPT_DIR/$relative_path"
    if [[ ! -f "$path" ]]; then
        fail "Missing $relative_path"
    elif [[ "$path" == *.onnx && $(stat -c %s "$path") -lt 100000 ]]; then
        fail "$relative_path looks like a Git LFS pointer"
    else
        pass "$relative_path"
    fi
done

for header in /usr/include/zmq.hpp /usr/include/msgpack.hpp /usr/include/nlohmann/json.hpp; do
    [[ -f "$header" ]] && pass "$header" || fail "Missing $header"
done

if [[ -f "$SCRIPT_DIR/thirdparty/onnxruntime/include/onnxruntime_cxx_api.h" ]]; then
    pass "Local ONNX Runtime 1.16.3 is installed"
elif [[ -f /opt/onnxruntime/include/onnxruntime_cxx_api.h ]]; then
    pass "System ONNX Runtime is installed"
else
    fail "ONNX Runtime headers were not found"
fi

if (( errors > 0 )); then
    echo "$errors preflight check(s) failed." >&2
    exit 1
fi

echo "$TARGET_ARCH deployment preflight checks passed."
