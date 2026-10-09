#!/usr/bin/env bash
set -euo pipefail

# Corrects what's staged in every participant's academy/day_3 and
# academy/day_4 folders.
#
#   Day 3: only the four SIMPLIFIED task notebooks (first/second/third/
#          fourth_task_day3, no _unsolved suffix in the destination) plus
#          their intro pages are staged. The old, harder task8-task11
#          notebooks (hl_arm_gestures, ll_control, ik_control,
#          brainco_revo2_open_close, dex3_gradual_gripping) and their intro
#          pages are removed if present.
#
#   Day 4: the five chatbot/SLAM tasks (old naming: task12-task16, excluding
#          the brainco variant of task16) are staged renamed as
#          first..fifth_task_day4 and rewritten to talk to a local Ollama
#          server via ollama_api_util.py instead of the OpenAI API -- no API
#          key needed, just a running `ollama serve` with the chat/vision
#          models pulled. The old task12-task16 files (every variant,
#          including the brainco one, and openai_api_util.py) are removed if
#          present.
#
# Both days also get the corrected sdk_wrapper.py from academy/ (never the
# stale historical copies under staging_day3//staging_day4/, see
# restage_all.sh): the dex3 hands are wired backwards on the physical robot,
# so HAND_CMD_TOPICS/HAND_STATE_TOPICS now swap "left"/"right" before
# talking to the robot -- open_dex3_hand("left")/close_dex3_hand("left")
# move the left hand. The joint-limit math (HAND_MAX/MIN/CLOSED/OPEN) is
# untouched, so this only changes which wire a side's command goes out on.
#
#   - teilnehmer1..NUM_USERS -> UNSOLVED notebooks (the exercise versions)
#   - SOLVED_USER (teilnehmer13) -> SOLVED notebooks
#
# WARNING: this DELETES the current contents of each day_3/day_4 folder
# (including any arm_sequences.json / slam_points.json a participant
# recorded there).
#
# Usage:
#   sudo bash restage_day3_day4.sh
#   sudo NUM_USERS=12 SOLVED_USER=teilnehmer13 bash restage_day3_day4.sh

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_USER="${SOLVED_USER:-teilnehmer13}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
STAGING_DAY3="$SOLVED_DIR/staging_day3"
STAGING_DAY4="$SOLVED_DIR/staging_day4"
# Always the current, corrected wrapper -- never the historical copies
# checked into staging_day3//staging_day4/ (see restage_all.sh's comment).
CURRENT_SDK_WRAPPER="$ACADEMY_DIR/sdk_wrapper.py"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }

# ---- Day 3: easy first/second/third/fourth_task_day3 only -----------------
day3_deps=(
  "$CURRENT_SDK_WRAPPER"
  "$ACADEMY_DIR/util.py"
  "$ACADEMY_DIR/slam_util.py"
)
day3_intros=(
  "$STAGING_DAY3/first_task_day3_intro.html"
  "$STAGING_DAY3/second_task_day3_intro.html"
  "$STAGING_DAY3/third_task_day3_intro.html"
  "$STAGING_DAY3/fourth_task_day3_intro.html"
)
day3_unsolved_pairs=(
  "$SOLVED_DIR/first_task_day3_unsolved.ipynb::first_task_day3.ipynb"
  "$SOLVED_DIR/second_task_day3_unsolved.ipynb::second_task_day3.ipynb"
  "$SOLVED_DIR/third_task_day3_unsolved.ipynb::third_task_day3.ipynb"
  "$SOLVED_DIR/fourth_task_day3_unsolved.ipynb::fourth_task_day3.ipynb"
)
day3_solved_pairs=(
  "$SOLVED_DIR/first_task_day3.ipynb::first_task_day3.ipynb"
  "$SOLVED_DIR/second_task_day3.ipynb::second_task_day3.ipynb"
  "$SOLVED_DIR/third_task_day3.ipynb::third_task_day3.ipynb"
  "$SOLVED_DIR/fourth_task_day3_unsolved.ipynb::fourth_task_day3.ipynb"
)
# Difficult Day 3 notebooks + intros that must NOT be staged; removed from
# the destination if a previous staging left them behind.
day3_difficult_names=(
  task8_hl_arm_gestures.ipynb task8_hl_arm_gestures_intro.html
  task9_ll_control.ipynb task9_ll_control_intro.html
  task10_ik_control.ipynb task10_ik_control_intro.html
  task11_brainco_revo2_open_close.ipynb task11_brainco_revo2_open_close_intro.html
  task11_dex3_gradual_gripping.ipynb task11_dex3_gradual_gripping_intro.html
)

# ---- Day 4: easy first..fifth_task_day4, Ollama-based ---------------------
day4_deps=(
  "$CURRENT_SDK_WRAPPER"
  "$ACADEMY_DIR/util.py"
  "$ACADEMY_DIR/slam_util.py"
  "$STAGING_DAY4/ollama_api_util.py"
  "$STAGING_DAY4/g1_academy_knowledge_de.json"
  "$STAGING_DAY4/right_arm_forward_pose.json"
)
day4_intros=(
  "$STAGING_DAY4/first_task_day4_intro.html"
  "$STAGING_DAY4/second_task_day4_intro.html"
  "$STAGING_DAY4/third_task_day4_intro.html"
  "$STAGING_DAY4/fourth_task_day4_intro.html"
  "$STAGING_DAY4/fifth_task_day4_intro.html"
)
day4_unsolved_pairs=(
  "$STAGING_DAY4/first_task_day4_unsolved.ipynb::first_task_day4.ipynb"
  "$STAGING_DAY4/second_task_day4_unsolved.ipynb::second_task_day4.ipynb"
  "$STAGING_DAY4/third_task_day4_unsolved.ipynb::third_task_day4.ipynb"
  "$STAGING_DAY4/fourth_task_day4_unsolved.ipynb::fourth_task_day4.ipynb"
  "$STAGING_DAY4/fifth_task_day4_unsolved.ipynb::fifth_task_day4.ipynb"
)
day4_solved_pairs=(
  "$STAGING_DAY4/first_task_day4.ipynb::first_task_day4.ipynb"
  "$STAGING_DAY4/second_task_day4.ipynb::second_task_day4.ipynb"
  "$STAGING_DAY4/third_task_day4.ipynb::third_task_day4.ipynb"
  "$STAGING_DAY4/fourth_task_day4.ipynb::fourth_task_day4.ipynb"
  "$STAGING_DAY4/fifth_task_day4.ipynb::fifth_task_day4.ipynb"
)
# Difficult/retired Day 4 files (old OpenAI-based task12-16 naming, plus the
# brainco variant of task16, plus the now-unused OpenAI util) that must NOT
# be staged; removed from the destination if present.
day4_difficult_names=(
  task12_openai_chatbot_basic.ipynb task12_openai_chatbot_basic_unsolved.ipynb task12_openai_chatbot_basic_intro.html
  task13_openai_chatbot_rag.ipynb task13_openai_chatbot_rag_unsolved.ipynb task13_openai_chatbot_rag_intro.html
  task14_openai_chatbot_gestures_and_movement.ipynb task14_openai_chatbot_gestures_and_movement_unsolved.ipynb task14_openai_chatbot_gestures_and_movement_intro.html
  task15_openai_chatbot_vision.ipynb task15_openai_chatbot_vision_unsolved.ipynb task15_openai_chatbot_vision_intro.html
  task16_slam_pickup_delivery.ipynb task16_slam_pickup_delivery_unsolved.ipynb task16_slam_pickup_delivery_intro.html
  task16_slam_pickup_delivery_brainco.ipynb task16_slam_pickup_delivery_brainco_unsolved.ipynb task16_slam_pickup_delivery_brainco_intro.html
  openai_api_util.py
)

require_files() {
  local f
  for f in "$@"; do
    [[ -f "$f" ]] || { echo "Missing source: $f" >&2; exit 1; }
  done
}
require_files "${day3_deps[@]}" "${day3_intros[@]}" "${day4_deps[@]}" "${day4_intros[@]}"
for pair in "${day3_unsolved_pairs[@]}" "${day3_solved_pairs[@]}" "${day4_unsolved_pairs[@]}" "${day4_solved_pairs[@]}"; do
  require_files "${pair%%::*}"
done

restage_day3_user() {
  local user="$1"; shift
  local pairs=("$@")
  local home_dir dest name
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  dest="$home_dir/academy/day_3"
  rm -rf "$dest"
  install -d -m 0755 "$dest"
  for pair in "${pairs[@]}"; do
    cp "${pair%%::*}" "$dest/${pair##*::}"
  done
  cp "${day3_intros[@]}" "${day3_deps[@]}" "$dest/"
  for name in "${day3_difficult_names[@]}"; do
    rm -f "$dest/$name"
  done
  chown -R "$user:$user" "$dest"
  echo "restaged $user day_3"
}

restage_day4_user() {
  local user="$1"; shift
  local pairs=("$@")
  local home_dir dest name
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  dest="$home_dir/academy/day_4"
  rm -rf "$dest"
  install -d -m 0755 "$dest"
  for pair in "${pairs[@]}"; do
    cp "${pair%%::*}" "$dest/${pair##*::}"
  done
  cp "${day4_intros[@]}" "${day4_deps[@]}" "$dest/"
  for name in "${day4_difficult_names[@]}"; do
    rm -f "$dest/$name"
  done
  chown -R "$user:$user" "$dest"
  echo "restaged $user day_4"
}

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  restage_day3_user "$user" "${day3_unsolved_pairs[@]}"
  restage_day4_user "$user" "${day4_unsolved_pairs[@]}"
done
restage_day3_user "$SOLVED_USER" "${day3_solved_pairs[@]}"
restage_day4_user "$SOLVED_USER" "${day4_solved_pairs[@]}"

echo "Done. Restaged Day 3 (easy set only) and Day 4 (Ollama-based easy set) for $NUM_USERS participant(s) + $SOLVED_USER (solved)."
echo "Reminder: participants must restart their Jupyter kernel to load the corrected sdk_wrapper.py, and Day 4 needs a running local Ollama server (ollama serve) with the chat/vision models pulled."
