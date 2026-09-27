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

restage_user() {
  local user="$1"; shift
  local pairs=("$@")
  local home_dir dest
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  dest="$home_dir/academy/day_3"
  rm -rf "$dest"
  install -d -m 0755 "$dest"
  for pair in "${pairs[@]}"; do
    cp "${pair%%::*}" "$dest/${pair##*::}"
  done
  cp "${intros[@]}" "${deps[@]}" "$dest/"
  chown -R "$user:$user" "$dest"
  echo "restaged $user -> $dest"
}

for ((i=1; i<=NUM_USERS; i++)); do
  restage_user "${USER_PREFIX}${i}" "${unsolved_pairs[@]}"
done
restage_user "$SOLVED_USER" "${solved_pairs[@]}"

echo "Done. Restaged Day 3 for $NUM_USERS participant(s) + $SOLVED_USER (solved)."
echo "Reminder: participants must restart their Jupyter kernel to load the new sdk_wrapper.py."
