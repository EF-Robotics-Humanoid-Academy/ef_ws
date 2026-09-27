#!/usr/bin/env bash
set -euo pipefail

# Targeted restage of Day 4 Task 16 (SLAM pickup & delivery) ONLY: refreshes
# the task16 notebook, its intro page, and the updated modules it needs, in
# each participant's existing academy/day_4 folder. It does NOT wipe day_4 or
# touch the other Day 4 tasks.
#
#   - teilnehmer1..NUM_USERS  -> UNSOLVED task16 notebook
#   - SOLVED_USER (teilnehmer13) -> SOLVED task16 notebook
#
# In every folder the notebook is named task16_slam_pickup_delivery.ipynb (the
# _unsolved suffix is dropped on copy), so both sets open the same filename.
# Modules staged: sdk_wrapper.py (stop_mapping self-heal) and openai_api_util.py
# (now provides detect_object), plus the right_arm_forward_pose.json asset.
#
# A user whose academy/day_4 folder does not exist yet is skipped (this script
# updates existing Day 4 bundles, it does not create partial ones).
#
# Usage:
#   sudo bash restage_task16.sh
#   sudo NUM_USERS=12 SOLVED_USER=teilnehmer13 bash restage_task16.sh

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_USER="${SOLVED_USER:-teilnehmer13}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
STAGING_DIR="$SOLVED_DIR/staging_day4"

NB_NAME="task16_slam_pickup_delivery.ipynb"
UNSOLVED_NB="$STAGING_DIR/task16_slam_pickup_delivery_unsolved.ipynb"
SOLVED_NB="$STAGING_DIR/task16_slam_pickup_delivery.ipynb"
INTRO="$STAGING_DIR/task16_slam_pickup_delivery_intro.html"
POSE="$STAGING_DIR/right_arm_forward_pose.json"

# Shared files copied verbatim (same name) into every day_4 folder.
deps=(
  "$ACADEMY_DIR/sdk_wrapper.py"
  "$ACADEMY_DIR/openai_api_util.py"
  "$INTRO"
  "$POSE"
)

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
for f in "$UNSOLVED_NB" "$SOLVED_NB" "${deps[@]}"; do
  [[ -f "$f" ]] || { echo "Missing source: $f" >&2; exit 1; }
done

update_user() {
  local user="$1" notebook="$2"
  local home_dir dest
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  dest="$home_dir/academy/day_4"
  if [[ ! -d "$dest" ]]; then
    echo "skip $user: no $dest (Day 4 not staged yet)"
    return 0
  fi
  cp "$notebook" "$dest/$NB_NAME"
  cp "${deps[@]}" "$dest/"
  chown "$user:$user" "$dest/$NB_NAME" \
    "$dest/sdk_wrapper.py" "$dest/openai_api_util.py" \
    "$dest/task16_slam_pickup_delivery_intro.html" "$dest/right_arm_forward_pose.json"
  echo "updated $user -> $dest/$NB_NAME"
}

for ((i=1; i<=NUM_USERS; i++)); do
  update_user "${USER_PREFIX}${i}" "$UNSOLVED_NB"
done
update_user "$SOLVED_USER" "$SOLVED_NB"

echo "Task 16 restaged. Reminder: participants must restart their Jupyter kernel to load the new sdk_wrapper.py / openai_api_util.py."
