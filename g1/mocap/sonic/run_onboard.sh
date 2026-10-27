#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
TARGET_ARCH="$(package_arch "$SCRIPT_DIR")"

require_matching_package_arch "$SCRIPT_DIR"

usage() {
    echo "Usage: $0 <dds_interface> <gmr_operator_ip> [zmq_port] [release|low_latency] [real|sim] [--zmq-conflate]"
    echo "Real example: $0 eth0 192.168.3.10 5556 release real"
    echo "Simulation example: $0 lo localhost 5556 release sim"
}

if [[ $# -lt 2 || $# -gt 6 ]]; then
    usage >&2
    exit 2
fi

robot_interface="$1"
gmr_host="$2"
zmq_port="${3:-5556}"
model_profile="${4:-release}"
runtime_target="${5:-real}"
conflate="${6:-}"

if [[ ! -d "/sys/class/net/$robot_interface" ]]; then
    echo "Error: network interface '$robot_interface' does not exist." >&2
    exit 1
fi
if [[ ! "$zmq_port" =~ ^[0-9]+$ ]] || (( zmq_port < 1 || zmq_port > 65535 )); then
    echo "Error: invalid ZMQ port '$zmq_port'." >&2
    exit 1
fi
if [[ "$model_profile" != "release" && "$model_profile" != "low_latency" ]]; then
    echo "Error: model profile must be 'release' or 'low_latency'." >&2
    exit 1
fi
if [[ "$runtime_target" != "real" && "$runtime_target" != "sim" ]]; then
    echo "Error: runtime target must be 'real' or 'sim'." >&2
    exit 1
fi
if [[ -n "$conflate" && "$conflate" != "--zmq-conflate" ]]; then
    echo "Error: unsupported argument '$conflate'." >&2
    exit 1
fi
if [[ "$runtime_target" == "sim" && "$TARGET_ARCH" != "x86_64" ]]; then
    echo "Error: MuJoCo simulation is supported on Ubuntu x86_64 only." >&2
    exit 1
fi

binary="$SCRIPT_DIR/bin/g1_deploy_onnx_ref"
if [[ ! -x "$binary" ]]; then
    echo "Error: $binary is missing. Run ./build_onboard.sh first." >&2
    exit 1
fi

onnx_root="${onnxruntime_ROOT:-}"
if [[ -z "$onnx_root" && -d "$SCRIPT_DIR/thirdparty/onnxruntime" ]]; then
    onnx_root="$SCRIPT_DIR/thirdparty/onnxruntime"
elif [[ -z "$onnx_root" && -d /opt/onnxruntime ]]; then
    onnx_root=/opt/onnxruntime
fi
sdk_lib="$SCRIPT_DIR/thirdparty/unitree_sdk2/thirdparty/lib/$TARGET_ARCH"
export LD_LIBRARY_PATH="$sdk_lib:$onnx_root/lib:${TensorRT_ROOT:-/usr}/lib:${LD_LIBRARY_PATH:-}"

if [[ "$runtime_target" == "real" && "${G1_DEPLOY_CONFIRMED:-0}" != "1" ]]; then
    echo "REAL ROBOT CONTROL: keep the G1 suspended and keep an emergency stop operator present."
    read -r -p "Type RUN to launch the controller: " confirmation
    if [[ "$confirmation" != "RUN" ]]; then
        echo "Cancelled."
        exit 0
    fi
fi

extra_args=()
if [[ "$runtime_target" == "sim" ]]; then
    extra_args+=(--disable-crc-check)
fi
if [[ "$conflate" == "--zmq-conflate" ]]; then
    extra_args+=(--zmq-conflate)
fi

cd "$SCRIPT_DIR"
exec "$binary" \
    "$robot_interface" \
    "policy/$model_profile/model_decoder.onnx" \
    reference/ \
    --obs-config "policy/$model_profile/observation_config.yaml" \
    --encoder-file "policy/$model_profile/model_encoder.onnx" \
    --input-type zmq \
    --output-type zmq \
    --zmq-host "$gmr_host" \
    --zmq-port "$zmq_port" \
    --zmq-topic pose \
    "${extra_args[@]}"
