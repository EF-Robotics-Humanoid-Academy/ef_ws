#!/usr/bin/env bash
set -euo pipefail

# Rebuild every managed academy day from the checked-in staging/reference
# material.  This deliberately removes each day directory first, so stale
# notebooks, generated files, and old documentation cannot survive a restage.
#
# teilnehmer1..NUM_USERS receive the exercise notebooks.  SOLVED_USER receives
# the solved reference notebooks for every day; all users receive the matching
# slides, task introductions, dependencies, and shared academy/docs images.
#
# Usage:
#   sudo bash restage_all.sh
#   sudo NUM_USERS=12 SOLVED_USER=teilnehmer13 bash restage_all.sh

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_USER="${SOLVED_USER:-teilnehmer13}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
# Always stage this authoritative, current wrapper.  Do not use the historical
# copies kept inside staging_day*/.
CURRENT_SDK_WRAPPER="$ACADEMY_DIR/sdk_wrapper.py"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || {
  echo "NUM_USERS must be 1..100" >&2; exit 2;
}
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] || { echo "Unsafe USER_PREFIX" >&2; exit 2; }
command -v rsync >/dev/null || { echo "rsync is required" >&2; exit 1; }

day1_unsolved=("$SOLVED_DIR/staging_day1"/task*.ipynb)
day1_solved=("$SOLVED_DIR"/task{1,2,3,4}_*.ipynb)
day2_unsolved=("$SOLVED_DIR/staging_day2"/task*.ipynb)
day2_solved=("$SOLVED_DIR"/task{5,6,7}_*.ipynb)
day3_unsolved=("$SOLVED_DIR/staging_day3"/task*.ipynb)
day3_solved=("$SOLVED_DIR"/task{8,9,10,11}_*.ipynb)
day4_unsolved=("$SOLVED_DIR/staging_day4"/task*_unsolved.ipynb)
day4_solved=(
  "$SOLVED_DIR/staging_day4/task12_openai_chatbot_basic.ipynb"
  "$SOLVED_DIR/staging_day4/task13_openai_chatbot_rag.ipynb"
  "$SOLVED_DIR/staging_day4/task14_openai_chatbot_gestures_and_movement.ipynb"
  "$SOLVED_DIR/staging_day4/task15_openai_chatbot_vision.ipynb"
  "$SOLVED_DIR/staging_day4/task16_slam_pickup_delivery.ipynb"
  "$SOLVED_DIR/staging_day4/task16_slam_pickup_delivery_brainco.ipynb"
)

require_files() {
  local file
  for file in "$@"; do
    [[ -f "$file" ]] || { echo "Missing source: $file" >&2; exit 1; }
  done
}

require_files \
  "${day1_unsolved[@]}" "${day1_solved[@]}" \
  "${day2_unsolved[@]}" "${day2_solved[@]}" \
  "${day3_unsolved[@]}" "${day3_solved[@]}" \
  "${day4_unsolved[@]}" "${day4_solved[@]}" \
  "$CURRENT_SDK_WRAPPER" "$ACADEMY_DIR/util.py" "$ACADEMY_DIR/slam_util.py" \
  "$ACADEMY_DIR/openai_api_util.py"
[[ -d "$SOLVED_DIR/imgs_real" ]] || { echo "Missing docs image source: $SOLVED_DIR/imgs_real" >&2; exit 1; }

stage_day() {
  local user="$1" day="$2" notebook_kind="$3"
  local home_dir destination staging_dir
  shift 3
  local notebooks=("$@")
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  destination="$home_dir/academy/day_$day"
  staging_dir="$SOLVED_DIR/staging_day$day"

  rm -rf "$destination"
  install -d -m 0755 "$destination"
  if [[ "$day" == 4 ]]; then
    local notebook name
    for notebook in "${notebooks[@]}"; do
      name="$(basename "$notebook")"
      name="${name%_unsolved.ipynb}.ipynb"
      cp "$notebook" "$destination/$name"
    done
  else
    rsync -a "${notebooks[@]}" "$destination/"
  fi
  rsync -a "$staging_dir"/*_intro.html "$staging_dir/day${day}_slides.html" \
    "$CURRENT_SDK_WRAPPER" "$ACADEMY_DIR/util.py" "$ACADEMY_DIR/slam_util.py" "$destination/"
  if [[ "$day" == 4 ]]; then
    rsync -a "$ACADEMY_DIR/openai_api_util.py" "$staging_dir/g1_academy_knowledge_de.json" \
      "$staging_dir/right_arm_forward_pose.json" "$destination/"
  fi
  chown -R "$user:$user" "$destination"
  echo "restaged $user day_$day ($notebook_kind)"
}

stage_docs() {
  local user="$1" home_dir docs_destination
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  docs_destination="$home_dir/academy/docs"
  rm -rf "$docs_destination"
  install -d -m 0755 "$docs_destination"
  rsync -a "$SOLVED_DIR/imgs_real/" "$docs_destination/imgs/"
  chmod -R a+rX "$docs_destination"
  chown -R "$user:$user" "$docs_destination"
}

stage_user() {
  local user="$1" solved=false
  [[ "$user" == "$SOLVED_USER" ]] && solved=true
  if $solved; then
    stage_day "$user" 1 solved "${day1_solved[@]}"
    stage_day "$user" 2 solved "${day2_solved[@]}"
    stage_day "$user" 3 solved "${day3_solved[@]}"
    stage_day "$user" 4 solved "${day4_solved[@]}"
  else
    stage_day "$user" 1 exercise "${day1_unsolved[@]}"
    stage_day "$user" 2 exercise "${day2_unsolved[@]}"
    stage_day "$user" 3 exercise "${day3_unsolved[@]}"
    stage_day "$user" 4 exercise "${day4_unsolved[@]}"
  fi
  stage_docs "$user"
}

for ((i=1; i<=NUM_USERS; i++)); do
  stage_user "${USER_PREFIX}${i}"
done
if (( NUM_USERS < 13 )) || [[ "${USER_PREFIX}13" != "$SOLVED_USER" ]]; then
  stage_user "$SOLVED_USER"
fi

echo "Done. Restaged all four days and shared docs; $SOLVED_USER received solved notebooks."
