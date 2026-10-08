#!/usr/bin/env bash
set -euo pipefail

# Stop all active participant Jupyter single-user servers.  JupyterHub records
# them as stopped and starts a fresh server when each participant next visits
# the Hub. This is the reliable "restart all" operation for LocalProcessSpawner.
# Usage: sudo bash restart_participant_jupyter_servers.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"

[[ "$(id -u)" -eq 0 ]] || {
  echo "Run as root: sudo bash $0" >&2
  exit 1
}
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || {
  echo "NUM_USERS must be 1..100" >&2
  exit 2
}
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || {
  echo "USER_PREFIX must be a non-empty, safe Linux username prefix (max 24 characters)" >&2
  exit 2
}

stopped=0
already_stopped=0
for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  if pkill -u "$user" -f jupyterhub-singleuser; then
    echo "Stopped Jupyter server for $user"
    ((stopped += 1))
  else
    echo "No active Jupyter server for $user"
    ((already_stopped += 1))
  fi
done

echo "Finished: stopped $stopped server(s); $already_stopped already stopped."
echo "Participants get a fresh server automatically on their next JupyterHub visit."
