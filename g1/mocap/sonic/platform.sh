#!/usr/bin/env bash

normalize_arch() {
    case "$1" in
        aarch64|arm64)
            echo aarch64
            ;;
        x86_64|amd64)
            echo x86_64
            ;;
        *)
            echo "$1"
            ;;
    esac
}

package_arch() {
    local package_dir="$1"
    local configured_arch=""
    if [[ -f "$package_dir/PACKAGE_INFO" ]]; then
        configured_arch="$(awk -F= '$1 == "target_arch" {print $2; exit}' "$package_dir/PACKAGE_INFO")"
    fi
    if [[ "$configured_arch" == "universal" || -z "$configured_arch" ]]; then
        normalize_arch "$(uname -m)"
    else
        normalize_arch "$configured_arch"
    fi
}

package_target_arch() {
    local package_dir="$1"
    if [[ -f "$package_dir/PACKAGE_INFO" ]]; then
        awk -F= '$1 == "target_arch" {print $2; exit}' "$package_dir/PACKAGE_INFO"
    else
        echo universal
    fi
}

expected_tensorrt_version() {
    case "$1" in
        aarch64)
            echo 10.7
            ;;
        x86_64)
            echo 10.13
            ;;
        *)
            return 1
            ;;
    esac
}

onnxruntime_archive_arch() {
    case "$1" in
        aarch64)
            echo aarch64
            ;;
        x86_64)
            echo x64
            ;;
        *)
            return 1
            ;;
    esac
}

require_matching_package_arch() {
    local package_dir="$1"
    local expected_arch
    local host_arch
    expected_arch="$(package_arch "$package_dir")"
    host_arch="$(normalize_arch "$(uname -m)")"
    if [[ "$host_arch" != "aarch64" && "$host_arch" != "x86_64" ]]; then
        echo "Error: unsupported host architecture $host_arch." >&2
        return 1
    fi
    if [[ "$host_arch" != "$expected_arch" ]]; then
        echo "Error: this package targets $expected_arch, but the current host is $host_arch." >&2
        return 1
    fi
}
