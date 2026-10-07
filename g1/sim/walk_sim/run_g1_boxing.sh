#!/usr/bin/env bash
# Start the simulator-only boxing controller with the selected CycloneDDS build.
set -euo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyclonedds_home="${UNITREE_CYCLONEDDS_HOME:-/home/unitree/cyclonedds_ws/install/cyclonedds}"
python_bin="${UNITREE_PYTHON:-$(command -v python3)}"
export LD_LIBRARY_PATH="${cyclonedds_home}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
export CYCLONEDDS_HOME="${cyclonedds_home}"

exec "${python_bin}" "${project_dir}/example/python/g1_boxing.py" "$@"
