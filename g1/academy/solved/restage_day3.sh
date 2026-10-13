#!/usr/bin/env bash
set -euo pipefail

# Wipe every participant's academy/day_3 folder and restage the simplified Day 3
# bundle from scratch: the four new Day 3 task notebooks, the updated
# sdk_wrapper.py, its deps, and the four task intro pages.
#
#   - teilnehmer1..NUM_USERS  -> UNSOLVED notebooks (the exercise versions)
#   - SOLVED_USER (teilnehmer13) -> SOLVED notebooks (Task 4 has no code answer,
#                                   so it gets the same open-ended notebook)
#
# In every folder the notebooks are named first/second/third/fourth_task_day3.ipynb
# (no _unsolved suffix) so participants and the solved account open the same names.
#
# WARNING: this DELETES the current contents of each day_3 folder (including any
# recorded arm_sequences.json / slam_points.json a participant created there).
#
# Usage:
#   sudo bash restage_day3.sh
#   sudo NUM_USERS=12 SOLVED_USER=teilnehmer13 bash restage_day3.sh

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_USER="${SOLVED_USER:-teilnehmer13}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
STAGING_DIR="$SOLVED_DIR/staging_day3"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
if [[ ! "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] || (( NUM_USERS > 100 )); then
  echo "NUM_USERS must be 1..100" >&2
  exit 2
fi
if [[ ! "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] || (( ${#USER_PREFIX} > 24 )); then
  echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2
  exit 2
fi
if [[ ! "$SOLVED_USER" =~ ^[a-z_][a-z0-9_-]*$ ]] || (( ${#SOLVED_USER} > 32 )); then
  echo "SOLVED_USER must be a safe Linux username" >&2
  exit 2
fi
for required_command in getent install mktemp cp mv chown; do
  command -v "$required_command" >/dev/null || { echo "$required_command is required" >&2; exit 1; }
done

temporary_dirs=()
cleanup_temporary_dirs() {
  local directory
  for directory in "${temporary_dirs[@]}"; do
    [[ -d "$directory" ]] && rm -rf -- "$directory"
  done
}
trap cleanup_temporary_dirs EXIT

# Shared, un-renamed files copied into every day_3 folder.
deps=(
  "$ACADEMY_DIR/sdk_wrapper.py"
  "$ACADEMY_DIR/util.py"
  "$ACADEMY_DIR/slam_util.py"
)
intros=(
  "$STAGING_DIR/first_task_day3_intro.html"
  "$STAGING_DIR/second_task_day3_intro.html"
  "$STAGING_DIR/third_task_day3_intro.html"
  "$STAGING_DIR/fourth_task_day3_intro.html"
)

# Notebook copies as "source::destination-name" so unsolved/solved land under the
# same four filenames in the participant folder.
unsolved_pairs=(
  "$SOLVED_DIR/first_task_day3_unsolved.ipynb::first_task_day3.ipynb"
  "$SOLVED_DIR/second_task_day3_unsolved.ipynb::second_task_day3.ipynb"
  "$SOLVED_DIR/third_task_day3_unsolved.ipynb::third_task_day3.ipynb"
  "$SOLVED_DIR/fourth_task_day3_unsolved.ipynb::fourth_task_day3.ipynb"
)
solved_pairs=(
  "$SOLVED_DIR/first_task_day3.ipynb::first_task_day3.ipynb"
  "$SOLVED_DIR/second_task_day3.ipynb::second_task_day3.ipynb"
  "$SOLVED_DIR/third_task_day3.ipynb::third_task_day3.ipynb"
  "$SOLVED_DIR/fourth_task_day3_unsolved.ipynb::fourth_task_day3.ipynb"
)

# Fail early if any source is missing.
for f in "${deps[@]}" "${intros[@]}"; do
  [[ -f "$f" ]] || { echo "Missing source: $f" >&2; exit 1; }
done
for pair in "${unsolved_pairs[@]}" "${solved_pairs[@]}"; do
  [[ -f "${pair%%::*}" ]] || { echo "Missing source notebook: ${pair%%::*}" >&2; exit 1; }
done

target_users=()
for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  [[ "$user" == "$SOLVED_USER" ]] || target_users+=("$user")
done
target_users+=("$SOLVED_USER")
for user in "${target_users[@]}"; do
  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  [[ "$home_dir" == /* && "$home_dir" != / && -d "$home_dir" ]] || {
    echo "Unsafe or missing home directory for $user: $home_dir" >&2
    exit 1
  }
done

restage_user() {
  local user="$1"; shift
  local pairs=("$@")
  local home_dir academy_dir dest temp_dir
  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  academy_dir="$home_dir/academy"
  dest="$home_dir/academy/day_3"
  [[ "$dest" == "$academy_dir/day_3" && "$dest" != / ]] || { echo "Unsafe destination: $dest" >&2; exit 1; }
  install -d -m 0755 "$academy_dir"
  temp_dir="$(mktemp -d "$academy_dir/.day_3.stage.XXXXXX")"
  temporary_dirs+=("$temp_dir")
  for pair in "${pairs[@]}"; do
    cp -- "${pair%%::*}" "$temp_dir/${pair##*::}"
  done
  cp -- "${intros[@]}" "${deps[@]}" "$temp_dir/"
  chown -R -- "$user:$user" "$temp_dir"
  rm -rf -- "$dest"
  mv -- "$temp_dir" "$dest"
  echo "restaged $user -> $dest"
}

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  [[ "$user" == "$SOLVED_USER" ]] && continue
  restage_user "$user" "${unsolved_pairs[@]}"
done
restage_user "$SOLVED_USER" "${solved_pairs[@]}"

echo "Done. Restaged Day 3 for $NUM_USERS participant(s) + $SOLVED_USER (solved)."
echo "Reminder: participants must restart their Jupyter kernel to load the new sdk_wrapper.py."
