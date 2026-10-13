#!/usr/bin/env bash
set -euo pipefail

# Stage the SOLVED (full reference-answer) notebooks for a day into one or more
# participant accounts -- unlike stage_later_day_materials.sh, notebook code
# cells are NOT stripped into TODOs. Intended for an instructor/verification
# account (e.g. teilnehmer13), not the whole cohort.
#
# Usage:
#   sudo bash stage_solved_materials.sh DAY [USER ...]
# Examples:
#   sudo bash stage_solved_materials.sh 2                 # -> teilnehmer13 (default)
#   sudo bash stage_solved_materials.sh 2 teilnehmer13
#   sudo bash stage_solved_materials.sh 2 teilnehmer1 teilnehmer13
#   sudo HAND_VARIANT=brainco bash stage_solved_materials.sh 3 teilnehmer13

day="${1:?Usage: sudo bash $0 DAY [USER ...] (DAY = 2, 3, or 4)}"
shift || true
users=("$@")
[[ ${#users[@]} -gt 0 ]] || users=("teilnehmer13")
HAND_VARIANT="${HAND_VARIANT:-dex3}"

SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
STAGING_DIR="$SOLVED_DIR/staging_day$day"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0 $day ${users[*]}" >&2; exit 1; }
case "$day" in
  2) tasks=(5 6 7) ;;
  3) tasks=(8 9 10 11) ;;
  4) tasks=(first second third fourth fifth) ;;
  *) echo "DAY must be 2, 3, or 4" >&2; exit 2 ;;
esac
[[ "$HAND_VARIANT" == "dex3" || "$HAND_VARIANT" == "brainco" ]] || {
  echo "HAND_VARIANT must be 'dex3' or 'brainco'" >&2
  exit 2
}
for required_command in getent install rsync chown; do
  command -v "$required_command" >/dev/null || {
    echo "Required command is not installed: $required_command" >&2
    exit 1
  }
done

# Full solved notebooks (copied verbatim, no stripping) + the day's intro pages.
notebooks=()
for task in "${tasks[@]}"; do
  source_dir="$SOLVED_DIR"
  if [[ "$day" == 4 ]]; then
    match=("$STAGING_DIR/${task}_task_day4.ipynb")
  elif [[ "$task" == 11 ]]; then
    match=("$source_dir"/task"$task"_*"$HAND_VARIANT"*.ipynb)
  else
    match=("$source_dir"/task"$task"_*.ipynb)
  fi
  # Solved references never use the participant-only _unsolved suffix.
  solved_matches=()
  for candidate in "${match[@]}"; do
    [[ "$candidate" == *_unsolved.ipynb ]] || solved_matches+=("$candidate")
  done
  match=("${solved_matches[@]}")
  (( ${#match[@]} == 1 )) || {
    echo "Expected exactly one solved Task $task notebook, found ${#match[@]}" >&2
    exit 1
  }
  [[ -f "${match[0]}" ]] || { echo "Missing solved Task $task notebook in $SOLVED_DIR" >&2; exit 1; }
  notebooks+=("${match[0]}")
done
intros=()
for task in "${tasks[@]}"; do
  if [[ "$day" == 4 ]]; then
    match=("$STAGING_DIR/${task}_task_day4_intro.html")
  elif [[ "$task" == 11 ]]; then
    match=("$STAGING_DIR"/task"$task"_*"$HAND_VARIANT"*_intro.html)
  else
    match=("$STAGING_DIR"/task"$task"_*_intro.html)
  fi
  (( ${#match[@]} == 1 )) || {
    echo "Expected exactly one Task $task intro page, found ${#match[@]}" >&2
    exit 1
  }
  [[ -f "${match[0]}" ]] || { echo "Missing Task $task intro page in $STAGING_DIR" >&2; exit 1; }
  intros+=("${match[0]}")
done
slides="$STAGING_DIR/day${day}_slides.html"
[[ -f "$slides" ]] || { echo "Missing $slides" >&2; exit 1; }
required_files=(
  "$ACADEMY_DIR/sdk_wrapper.py"
  "$ACADEMY_DIR/util.py"
  "$ACADEMY_DIR/slam_util.py"
)
if [[ "$day" == 4 ]]; then
  required_files+=(
    "$ACADEMY_DIR/ollama_api_util.py"
    "$STAGING_DIR/g1_academy_knowledge_de.json"
    "$STAGING_DIR/right_arm_forward_pose.json"
  )
fi
for required_file in "${required_files[@]}"; do
  [[ -f "$required_file" ]] || { echo "Missing dependency: $required_file" >&2; exit 1; }
done

for user in "${users[@]}"; do
  if [[ ! "$user" =~ ^[a-z_][a-z0-9_-]*$ ]] || (( ${#user} > 32 )); then
    echo "Unsafe participant username: $user" >&2
    exit 2
  fi
  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  [[ "$home_dir" == /* && "$home_dir" != / ]] || { echo "Unsafe home directory for $user: $home_dir" >&2; exit 1; }
  destination="$home_dir/academy/day_$day"
  install -d -m 0755 "$destination"
  rsync -a "${notebooks[@]}" "${intros[@]}" "$slides" \
    "$ACADEMY_DIR/sdk_wrapper.py" "$ACADEMY_DIR/util.py" "$ACADEMY_DIR/slam_util.py" \
    "$destination/"
  if [[ "$day" == 4 ]]; then
    rsync -a "$ACADEMY_DIR/ollama_api_util.py" \
      "$STAGING_DIR/g1_academy_knowledge_de.json" \
      "$STAGING_DIR/right_arm_forward_pose.json" "$destination/"
    rm -f -- "$destination/task12_openai_chatbot_basic.ipynb" \
      "$destination/task12_openai_chatbot_basic_unsolved.ipynb" \
      "$destination/task13_openai_chatbot_rag.ipynb" \
      "$destination/task13_openai_chatbot_rag_unsolved.ipynb" \
      "$destination/task14_openai_chatbot_gestures_and_movement.ipynb" \
      "$destination/task14_openai_chatbot_gestures_and_movement_unsolved.ipynb" \
      "$destination/task15_openai_chatbot_vision.ipynb" \
      "$destination/task15_openai_chatbot_vision_unsolved.ipynb" \
      "$destination/task16_slam_pickup_delivery.ipynb" \
      "$destination/task16_slam_pickup_delivery_unsolved.ipynb" \
      "$destination/task12_openai_chatbot_basic_intro.html" \
      "$destination/task13_openai_chatbot_rag_intro.html" \
      "$destination/task14_openai_chatbot_gestures_and_movement_intro.html" \
      "$destination/task15_openai_chatbot_vision_intro.html" \
      "$destination/task16_slam_pickup_delivery_intro.html" \
      "$destination/task16_slam_pickup_delivery_brainco.ipynb" \
      "$destination/task16_slam_pickup_delivery_brainco_unsolved.ipynb" \
      "$destination/task16_slam_pickup_delivery_brainco_intro.html" \
      "$destination/openai_api_util.py"
  fi
  chown -R -- "$user:$user" "$destination"
  echo "Staged SOLVED Day $day materials for $user at $destination"
done
