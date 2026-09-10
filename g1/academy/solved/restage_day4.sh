#!/usr/bin/env bash
set -euo pipefail

# Re-stage a clean Day 4 bundle for every participant, from scratch.
# Unlike stage_later_day_materials.sh (generic TODO-stripping of a single
# solved notebook), Day 4 ships pre-authored solved/unsolved notebook pairs
# (see staging_day4/task*_unsolved.ipynb) -- this script only copies files,
# it does not generate any notebook content itself.
#
# Usage: sudo bash restage_day4.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_MASTER="teilnehmer13"   # gets the solved task12/task16 notebooks instead of unsolved
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
STAGING_DIR="$SOLVED_DIR/staging_day4"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }

# tasks that ship a solved/unsolved pair; every other *_unsolved.ipynb in
# STAGING_DIR is picked up automatically below.
SOLVED_ONLY_FOR_MASTER=("task12_openai_chatbot_basic" "task16_slam_pickup_delivery")

# Shared dependency files every participant needs alongside the notebooks.
DEPS=(
  "$ACADEMY_DIR/sdk_wrapper.py"
  "$ACADEMY_DIR/util.py"
  "$ACADEMY_DIR/slam_util.py"
  "$ACADEMY_DIR/openai_api_util.py"
  "$STAGING_DIR/g1_academy_knowledge_de.json"
  "$STAGING_DIR/right_arm_forward_pose.json"
)
for f in "${DEPS[@]}"; do
  [[ -f "$f" ]] || { echo "Required Day 4 dependency is missing: $f" >&2; exit 1; }
done

# Every unsolved notebook in staging_day4/, and its task base name
# (task12_openai_chatbot_basic_unsolved.ipynb -> task12_openai_chatbot_basic).
unsolved_notebooks=("$STAGING_DIR"/task*_unsolved.ipynb)
[[ -f "${unsolved_notebooks[0]}" ]] || { echo "No task*_unsolved.ipynb found in $STAGING_DIR" >&2; exit 1; }

intro_pages=("$STAGING_DIR"/task*_intro.html)
day_slides="$STAGING_DIR/day4_slides.html"
[[ -f "$day_slides" ]] || { echo "Missing $day_slides" >&2; exit 1; }

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  destination="$home_dir/academy/day_4"

  # Empty the folder first so no stale file from an earlier staging survives.
  rm -rf "$destination"
  install -d -m 0755 "$destination"

  rsync -a "${DEPS[@]}" "$day_slides" "${intro_pages[@]}" "$destination/"

  for unsolved in "${unsolved_notebooks[@]}"; do
    base="$(basename "$unsolved" _unsolved.ipynb)"
    target="$destination/${base}.ipynb"
    is_solved_for_master=false
    if [[ "$user" == "$SOLVED_MASTER" ]]; then
      for solved_base in "${SOLVED_ONLY_FOR_MASTER[@]}"; do
        [[ "$base" == "$solved_base" ]] && is_solved_for_master=true
      done
    fi
    if $is_solved_for_master; then
      solved_source="$STAGING_DIR/${base}.ipynb"
      [[ -f "$solved_source" ]] || { echo "Missing solved notebook: $solved_source" >&2; exit 1; }
      cp "$solved_source" "$target"
    else
      cp "$unsolved" "$target"
    fi
  done

  chown -R "$user:$user" "$destination"
done

echo "Re-staged ~/academy/day_4 for $NUM_USERS participants ($SOLVED_MASTER got solved task12/task16)."
