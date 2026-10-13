#!/usr/bin/env bash
set -euo pipefail

# Rebuild every managed academy day from the checked-in staging/reference
# material.  This deliberately removes each day directory first, so stale
# notebooks, generated files, and old documentation cannot survive a restage.
# Day 3 uses the current simplified first..fourth task set; Day 4 uses the
# current first..fifth Ollama-based set rather than the retired OpenAI tasks.
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
for required_command in getent install mktemp cp mv rsync chown chmod; do
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

day1_unsolved=("$SOLVED_DIR/staging_day1"/task*.ipynb)
day1_solved=("$SOLVED_DIR"/task{1,2,3,4}_*.ipynb)
day2_unsolved=("$SOLVED_DIR/staging_day2"/task*.ipynb)
day2_solved=("$SOLVED_DIR"/task{5,6,7}_*.ipynb)
day3_unsolved=(
  "$SOLVED_DIR/first_task_day3_unsolved.ipynb"
  "$SOLVED_DIR/second_task_day3_unsolved.ipynb"
  "$SOLVED_DIR/third_task_day3_unsolved.ipynb"
  "$SOLVED_DIR/fourth_task_day3_unsolved.ipynb"
)
day3_solved=(
  "$SOLVED_DIR/first_task_day3.ipynb"
  "$SOLVED_DIR/second_task_day3.ipynb"
  "$SOLVED_DIR/third_task_day3.ipynb"
  "$SOLVED_DIR/fourth_task_day3_unsolved.ipynb"
)
day4_unsolved=("$SOLVED_DIR/staging_day4"/{first,second,third,fourth,fifth}_task_day4_unsolved.ipynb)
day4_solved=(
  "$SOLVED_DIR/staging_day4/first_task_day4.ipynb"
  "$SOLVED_DIR/staging_day4/second_task_day4.ipynb"
  "$SOLVED_DIR/staging_day4/third_task_day4.ipynb"
  "$SOLVED_DIR/staging_day4/fourth_task_day4.ipynb"
  "$SOLVED_DIR/staging_day4/fifth_task_day4.ipynb"
)
day1_intros=("$SOLVED_DIR/staging_day1"/task{1,2,3,4}_*_intro.html)
day2_intros=("$SOLVED_DIR/staging_day2"/task{5,6,7}_*_intro.html)
day3_intros=("$SOLVED_DIR/staging_day3"/{first,second,third,fourth}_task_day3_intro.html)
day4_intros=("$SOLVED_DIR/staging_day4"/{first,second,third,fourth,fifth}_task_day4_intro.html)

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
  "${day1_intros[@]}" "${day2_intros[@]}" "${day3_intros[@]}" "${day4_intros[@]}" \
  "$CURRENT_SDK_WRAPPER" "$ACADEMY_DIR/util.py" "$ACADEMY_DIR/slam_util.py" \
  "$ACADEMY_DIR/ollama_api_util.py" \
  "$SOLVED_DIR/staging_day4/g1_academy_knowledge_de.json" \
  "$SOLVED_DIR/staging_day4/right_arm_forward_pose.json" \
  "$SOLVED_DIR/staging_day1/day1_slides.html" "$SOLVED_DIR/staging_day2/day2_slides.html" \
  "$SOLVED_DIR/staging_day3/day3_slides.html" "$SOLVED_DIR/staging_day4/day4_slides.html"
[[ -d "$SOLVED_DIR/imgs_real" ]] || { echo "Missing docs image source: $SOLVED_DIR/imgs_real" >&2; exit 1; }

# Validate the complete cohort before deleting or replacing any staged data.
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

stage_day() {
  local user="$1" day="$2" notebook_kind="$3"
  local home_dir academy_dir destination staging_dir temp_dir
  local intro_files=()
  shift 3
  local notebooks=("$@")
  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  academy_dir="$home_dir/academy"
  destination="$home_dir/academy/day_$day"
  staging_dir="$SOLVED_DIR/staging_day$day"
  [[ "$destination" == "$academy_dir/day_$day" && "$destination" != / ]] || {
    echo "Unsafe destination: $destination" >&2; exit 1;
  }
  case "$day" in
    1) intro_files=("${day1_intros[@]}") ;;
    2) intro_files=("${day2_intros[@]}") ;;
    3) intro_files=("${day3_intros[@]}") ;;
    4) intro_files=("${day4_intros[@]}") ;;
  esac

  install -d -m 0755 "$academy_dir"
  temp_dir="$(mktemp -d "$academy_dir/.day_${day}.stage.XXXXXX")"
  temporary_dirs+=("$temp_dir")
  local notebook name
  for notebook in "${notebooks[@]}"; do
    name="$(basename "$notebook")"
    if [[ "$name" == *_unsolved.ipynb ]]; then
      name="${name%_unsolved.ipynb}.ipynb"
    fi
    cp -- "$notebook" "$temp_dir/$name"
  done
  rsync -a "${intro_files[@]}" "$staging_dir/day${day}_slides.html" \
    "$CURRENT_SDK_WRAPPER" "$ACADEMY_DIR/util.py" "$ACADEMY_DIR/slam_util.py" "$temp_dir/"
  if [[ "$day" == 4 ]]; then
    rsync -a "$ACADEMY_DIR/ollama_api_util.py" "$staging_dir/g1_academy_knowledge_de.json" \
      "$staging_dir/right_arm_forward_pose.json" "$temp_dir/"
  fi
  chown -R -- "$user:$user" "$temp_dir"
  rm -rf -- "$destination"
  mv -- "$temp_dir" "$destination"
  echo "restaged $user day_$day ($notebook_kind)"
}

stage_docs() {
  local user="$1" home_dir academy_dir docs_destination temp_dir
  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  academy_dir="$home_dir/academy"
  docs_destination="$home_dir/academy/docs"
  [[ "$docs_destination" == "$academy_dir/docs" && "$docs_destination" != / ]] || {
    echo "Unsafe docs destination: $docs_destination" >&2; exit 1;
  }
  install -d -m 0755 "$academy_dir"
  temp_dir="$(mktemp -d "$academy_dir/.docs.stage.XXXXXX")"
  temporary_dirs+=("$temp_dir")
  rsync -a "$SOLVED_DIR/imgs_real/" "$temp_dir/imgs/"
  chmod -R a+rX "$temp_dir"
  chown -R -- "$user:$user" "$temp_dir"
  rm -rf -- "$docs_destination"
  mv -- "$temp_dir" "$docs_destination"
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
