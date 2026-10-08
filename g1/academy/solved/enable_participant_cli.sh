#!/usr/bin/env bash
set -euo pipefail

# Make the shared Codex executable and global ROS 2 Foxy installation
# available in every existing academy participant's interactive shell.
# Usage: sudo bash enable_participant_cli.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
CODEX_SOURCE="${CODEX_SOURCE:-/home/unitree/.local/bin/codex}"
CODEX_BIN="${CODEX_BIN:-/usr/local/bin/codex}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/foxy/setup.bash}"
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
CYCLONEDDS_URI="${CYCLONEDDS_URI:-$SCRIPT_DIR/cyclonedds-participants.xml}"
UNITREE_ROS2_INSTALL="${UNITREE_ROS2_INSTALL:-/home/unitree/unitree_ros2/cyclonedds_ws/install}"
UNITREE_ROS2_SETUP="${UNITREE_ROS2_SETUP:-$UNITREE_ROS2_INSTALL/setup.bash}"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
[[ -x "$CODEX_SOURCE" ]] || { echo "Codex executable is missing: $CODEX_SOURCE" >&2; exit 1; }
[[ -r "$ROS_SETUP" ]] || { echo "ROS 2 setup file is missing: $ROS_SETUP" >&2; exit 1; }
[[ -r "$CYCLONEDDS_URI" ]] || { echo "CycloneDDS configuration is missing: $CYCLONEDDS_URI" >&2; exit 1; }
[[ -f "$UNITREE_ROS2_INSTALL/unitree_go/lib/libunitree_go__python.so" ]] || {
  echo "Unitree ROS 2 type-support library is missing below: $UNITREE_ROS2_INSTALL" >&2
  exit 1
}
[[ -r "$UNITREE_ROS2_SETUP" ]] || {
  echo "Unitree ROS 2 workspace setup file is missing: $UNITREE_ROS2_SETUP" >&2
  exit 1
}

# /usr/local/bin is in the standard PATH for every user. Preserve a Codex
# binary that is already installed there; otherwise expose the tested shared
# install with a symlink so upgrades are picked up automatically.
if [[ ! -x "$CODEX_BIN" ]]; then
  ln -s "$CODEX_SOURCE" "$CODEX_BIN"
fi

for ((i = 1; i <= NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  env_file="$home_dir/.g1-unitree-env"
  [[ -f "$env_file" ]] || { echo "Missing managed environment: $env_file" >&2; exit 1; }

  # Remove prior managed copies to keep this operation repeatable.
  sed -i '\|^# >>> academy CLI environment (managed) >>>$|,\|^# <<< academy CLI environment (managed) <<<$|d' "$env_file"
  cat >>"$env_file" <<EOF
# >>> academy CLI environment (managed) >>>
# ROS 2 must be sourced, not merely placed on PATH.  Source the *same
# workspace setup as unitree*, not only /opt/ros/foxy: it adds unitree_go,
# unitree_hg, unitree_api and the compatible CycloneDDS/RMW libraries.
# Without it, commands which deserialize Unitree messages (for example
# the ros2 topic echo /odommodestate command can load an incompatible
# type-support stack and crash.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$CYCLONEDDS_URI"
# Codex is available globally at $CODEX_BIN.
source "$ROS_SETUP"
source "$UNITREE_ROS2_SETUP"
# Keep the native Unitree message type-support libraries discoverable even
# when a shell inherited an incomplete loader path.  This intentionally
# repeats the workspace order established above, matching unitree's shell.
export LD_LIBRARY_PATH="$UNITREE_ROS2_INSTALL/unitree_hg/lib:$UNITREE_ROS2_INSTALL/unitree_go/lib:$UNITREE_ROS2_INSTALL/unitree_api/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export PYTHONPATH="$UNITREE_ROS2_INSTALL/unitree_hg/lib/python3.8/site-packages:$UNITREE_ROS2_INSTALL/unitree_go/lib/python3.8/site-packages:$UNITREE_ROS2_INSTALL/unitree_api/lib/python3.8/site-packages\${PYTHONPATH:+:\$PYTHONPATH}"
# <<< academy CLI environment (managed) <<<
EOF
  chown "$user:$user" "$env_file"
done

echo "Enabled Codex and ROS 2 for $NUM_USERS participant accounts."
