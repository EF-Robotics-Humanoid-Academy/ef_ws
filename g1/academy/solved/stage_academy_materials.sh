#!/usr/bin/env bash
set -euo pipefail

# Copy the academy materials (the parent of solved/) into each participant's
# home, but deliberately omit solved/. Remove only Desktop launcher files.
# Usage: sudo bash stage_academy_materials.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOURCE_DIR="$(cd "$(dirname "$0")/.." && pwd)"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
command -v rsync >/dev/null || { echo "rsync is required" >&2; exit 1; }

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  destination="$home_dir/academy"

  # Preserve students' own additions if this is run again.
  rsync -a --exclude='/solved/' "$SOURCE_DIR/" "$destination/"
  chown -R "$user:$user" "$destination"

  desktop_dir="$home_dir/Desktop"
  if [[ -d "$desktop_dir" ]]; then
    while IFS= read -r -d '' launcher; do
      rm -f -- "$launcher"
    done < <(find "$desktop_dir" -maxdepth 1 -type f -name '*.desktop' -print0)
  fi
done

echo "Staged academy materials for $NUM_USERS participants and removed their Desktop .desktop launchers."
