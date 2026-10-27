#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
ONNX_VERSION="1.16.3"
ONNX_DIR="$SCRIPT_DIR/thirdparty/onnxruntime"
TARGET_ARCH="$(package_arch "$SCRIPT_DIR")"
ONNX_ARCH="$(onnxruntime_archive_arch "$TARGET_ARCH")"
TRT_VERSION="$(expected_tensorrt_version "$TARGET_ARCH")"

require_matching_package_arch "$SCRIPT_DIR"

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    cmake \
    curl \
    libeigen3-dev \
    libmsgpack-dev \
    libzmq3-dev \
    nlohmann-json3-dev \
    pkg-config \
    zlib1g-dev

if [[ ! -f /usr/include/zmq.hpp ]]; then
    if apt-cache show cppzmq-dev >/dev/null 2>&1; then
        sudo apt-get install -y cppzmq-dev
    elif apt-cache show libcppzmq-dev >/dev/null 2>&1; then
        sudo apt-get install -y libcppzmq-dev
    else
        echo "Error: neither cppzmq-dev nor libcppzmq-dev is available." >&2
        exit 1
    fi
fi

if [[ ! -f "$ONNX_DIR/include/onnxruntime_cxx_api.h" ]]; then
    tmp_dir="$(mktemp -d)"
    trap 'rm -rf "$tmp_dir"' EXIT
    archive="$tmp_dir/onnxruntime.tgz"
    archive_dir="onnxruntime-linux-${ONNX_ARCH}-${ONNX_VERSION}"
    url="https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/${archive_dir}.tgz"

    echo "Downloading ONNX Runtime ${ONNX_VERSION} for ${TARGET_ARCH}..."
    curl --fail --location --retry 3 "$url" --output "$archive"
    tar -xzf "$archive" -C "$tmp_dir"
    mkdir -p "$(dirname "$ONNX_DIR")"
    mv "$tmp_dir/$archive_dir" "$ONNX_DIR"
fi

if [[ "$TARGET_ARCH" == "aarch64" ]]; then
    echo "Dependencies are ready. TensorRT $TRT_VERSION and CUDA must come from JetPack 6."
else
    echo "Dependencies are ready. Install CUDA and TensorRT $TRT_VERSION separately on the x86_64 host."
fi
