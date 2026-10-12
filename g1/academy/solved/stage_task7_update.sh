#!/usr/bin/env bash
set -euo pipefail

# One-off restage for the Task 7 SLAM fix: pushes the updated Task 7 notebook,
# its intro page, and the updated sdk_wrapper.py into each participant's
# academy/day_2 folder -- and NOTHING else. Participants get the unsolved
# notebook; the solved user (teilnehmer13) gets the solved notebook.
#
# Why sdk_wrapper.py: the notebooks import it from their own day_2 folder
# (~/academy/day_2/sdk_wrapper.py), so the wrapper change (stop_mapping() now
# saves to a hardcoded path; longer SLAM RPC timeout) must be copied there too.
#
# Usage:
#   sudo bash stage_task7_update.sh
#   sudo NUM_USERS=12 SOLVED_USER=teilnehmer13 bash stage_task7_update.sh
#
# After running, participants must RESTART their Jupyter kernel so the new
# sdk_wrapper.py is re-imported.

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_USER="${SOLVED_USER:-teilnehmer13}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"

NB="task7_slam_operation_and_map_visualization.ipynb"
INTRO="task7_slam_operation_and_map_visualization_intro.html"
UNSOLVED_NB="$SOLVED_DIR/staging_day2/$NB"
SOLVED_NB="$SOLVED_DIR/$NB"
INTRO_SRC="$SOLVED_DIR/staging_day2/$INTRO"
WRAPPER="$ACADEMY_DIR/sdk_wrapper.py"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
for f in "$UNSOLVED_NB" "$SOLVED_NB" "$INTRO_SRC" "$WRAPPER"; do
  [[ -f "$f" ]] || { echo "Missing source: $f" >&2; exit 1; }
done

# Push notebook + intro + wrapper into one user's day_2 folder and chown them.
stage_to() {
  local user="$1" notebook="$2"
  local home_dir destination
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  destination="$home_dir/academy/day_2"
  install -d -m 0755 "$destination"
  rsync -a "$notebook" "$INTRO_SRC" "$WRAPPER" "$destination/"
  chown "$user:$user" "$destination/$NB" "$destination/$INTRO" "$destination/sdk_wrapper.py"
  echo "updated $user -> $destination"
}

for ((i=1; i<=NUM_USERS; i++)); do
  stage_to "${USER_PREFIX}${i}" "$UNSOLVED_NB"
done
stage_to "$SOLVED_USER" "$SOLVED_NB"

echo "Task 7 update staged to $NUM_USERS participant(s) + $SOLVED_USER (solved)."
echo "Reminder: participants must restart their Jupyter kernel to pick up the new sdk_wrapper.py."
