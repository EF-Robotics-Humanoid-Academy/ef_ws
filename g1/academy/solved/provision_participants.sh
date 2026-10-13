#!/usr/bin/env bash
set -euo pipefail

# Provision the academy accounts without duplicating the 1.8 GB, tested
# Unitree Python runtime.  Each participant owns their SDK checkout, shell
# configuration and Jupyter kernel; the immutable runtime is shared read-only
# from the unitree account.
#
# Usage: sudo ./provision_participants.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
PASSWORD="${TEILNEHMER_PASSWORD:-academy2026}"
REF_PY="${REF_PY:-/home/unitree/.guv/envs/unitree}"
REF_SDK="${REF_SDK:-/home/unitree/unitree_sdk2_python}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
UNITREE_ROS2_INSTALL="${UNITREE_ROS2_INSTALL:-/home/unitree/unitree_ros2/cyclonedds_ws/install}"
UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$UNITREE_ROS2_INSTALL/setup.bash}"
CODEX_BIN="${CODEX_BIN:-/usr/local/bin/codex}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CYCLONEDDS_URI="${CYCLONEDDS_URI:-$SCRIPT_DIR/cyclonedds-participants.xml}"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run as root: sudo $0" >&2
  exit 1
fi
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] || { echo "Unsafe USER_PREFIX" >&2; exit 2; }
[[ -x "$REF_PY/bin/python" && -d "$REF_SDK/unitree_sdk2py" && -f "$CYCLONEDDS_HOME/lib/libddsc.so" && -r "$ROS_SETUP" && -r "$UNITREE_ROS2_SETUP" && -x "$CODEX_BIN" ]] || {
  echo "Reference SDK, Python runtime, CycloneDDS, ROS 2, or Codex is missing." >&2
  exit 1
}

# Refuse to provision a broken reference installation.
LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  PYTHONPATH="$REF_SDK" "$REF_PY/bin/python" -c 'import cyclonedds, unitree_sdk2py' || {
  echo "Reference Python environment cannot import CycloneDDS and Unitree SDK2." >&2
  exit 1
}

for ((i = 1; i <= NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  if ! id "$user" &>/dev/null; then
    useradd --create-home --shell /bin/bash "$user"
  fi
  for group in sudo wheel admin; do
    gpasswd --delete "$user" "$group" >/dev/null 2>&1 || true
  done
  echo "$user:$PASSWORD" | chpasswd

  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  sdk_dir="$home_dir/unitree_sdk2_python"
  rsync -a --delete --exclude=.git "$REF_SDK/" "$sdk_dir/"

  # The SDK's interface configuration otherwise writes one shared,
  # user-owned /tmp/cdds.LOG file.  Use a private trace file per account.
  dds_log_dir="$home_dir/.cache/cyclonedds"
  install -d -m 0700 "$dds_log_dir"
  channel_config="$sdk_dir/unitree_sdk2py/core/channel_config.py"
  [[ -f "$channel_config" ]] || { echo "Missing SDK channel config: $channel_config" >&2; exit 1; }
  sed -i "s|/tmp/cdds\.LOG|$dds_log_dir/cdds.LOG|g" "$channel_config"
  chown "$user:$user" "$dds_log_dir"

  cat >"$home_dir/.g1-unitree-env" <<EOF
# Managed by provision_participants.sh.  The runtime is deliberately shared
# read-only; this account owns the SDK source above and needs no sudo access.
export UNITREE_SDK2_HOME="$sdk_dir"
export CYCLONEDDS_HOME="$CYCLONEDDS_HOME"
export CycloneDDS_ROOT="\$CYCLONEDDS_HOME"
export PATH="$REF_PY/bin:\$CYCLONEDDS_HOME/bin:\${PATH}"
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export CMAKE_PREFIX_PATH="\$CYCLONEDDS_HOME\${CMAKE_PREFIX_PATH:+:\$CMAKE_PREFIX_PATH}"
export PKG_CONFIG_PATH="\$CYCLONEDDS_HOME/lib/pkgconfig\${PKG_CONFIG_PATH:+:\$PKG_CONFIG_PATH}"
export PYTHONPATH="\$UNITREE_SDK2_HOME\${PYTHONPATH:+:\$PYTHONPATH}"
# Match unitree's complete ROS workspace.  The workspace contributes the
# Unitree message packages and the matching CycloneDDS/RMW libraries; sourcing
# only /opt/ros/foxy is insufficient for ros2 topic echo /odommodestate.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CYCLONEDDS_URI"
source "$ROS_SETUP"
source "$UNITREE_ROS2_SETUP"
EOF
  touch "$home_dir/.bashrc"
  grep -qF '.g1-unitree-env' "$home_dir/.bashrc" || printf '\n[ -f "$HOME/.g1-unitree-env" ] && source "$HOME/.g1-unitree-env"\n' >>"$home_dir/.bashrc"

  kernel_dir="$home_dir/.local/share/jupyter/kernels/unitree_sdk2"
  install -d -m 0755 "$kernel_dir"
  cat >"$kernel_dir/kernel.json" <<EOF
{
  "argv": ["$REF_PY/bin/python", "-m", "ipykernel_launcher", "-f", "{connection_file}"],
  "display_name": "Unitree SDK2 (g1 academy)",
  "language": "python",
  "env": {
    "UNITREE_SDK2_HOME": "$sdk_dir",
    "CYCLONEDDS_HOME": "$CYCLONEDDS_HOME",
    "CycloneDDS_ROOT": "$CYCLONEDDS_HOME",
    "LD_LIBRARY_PATH": "$CYCLONEDDS_HOME/lib",
    "PYTHONPATH": "$sdk_dir"
  }
}
EOF
  chown -R "$user:$user" "$home_dir/unitree_sdk2_python" "$home_dir/.g1-unitree-env" "$home_dir/.bashrc" "$home_dir/.local"
done

echo "Provisioned $NUM_USERS accounts. Their Linux accounts have no sudo, wheel, or admin membership."
