#!/usr/bin/env bash
set -euo pipefail

# Re-stage a clean Day 4 bundle for every participant, from scratch.
#
# Day 4 ships the five simplified chatbot/SLAM tasks (renamed from the old
# task12-task16 OpenAI naming to first..fifth_task_day4) rewritten to talk
# to a local Ollama server via ollama_api_util.py instead of the OpenAI API
# -- no API key needed, just a running `ollama serve` on 127.0.0.1:11434
# with the chat/vision models pulled (qwen3.5:9b / qwen2.5vl:7b by
# default). The old task12-task16 files (every variant, including the
# brainco variant of task16, and the now-unused openai_api_util.py) are
# removed from the destination if present -- see restage_day3_day4.sh's
# comment for the full day3+day4 rationale, and use that script instead if
# you also want Day 3 re-staged in the same run.
#
#   - teilnehmer1..NUM_USERS -> UNSOLVED notebooks (the exercise versions)
#   - SOLVED_USER (teilnehmer13) -> SOLVED notebooks
#
# WARNING: this DELETES the current contents of each day_4 folder.
#
# Usage:
#   sudo bash restage_day4.sh
#   sudo NUM_USERS=12 SOLVED_USER=teilnehmer13 bash restage_day4.sh

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_USER="${SOLVED_USER:-teilnehmer13}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
STAGING_DIR="$SOLVED_DIR/staging_day4"
# Always the current, corrected wrapper -- never the historical copy
# checked into staging_day4/ (see restage_all.sh's comment).
CURRENT_SDK_WRAPPER="$ACADEMY_DIR/sdk_wrapper.py"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }

deps=(
  "$CURRENT_SDK_WRAPPER"
  "$ACADEMY_DIR/util.py"
  "$ACADEMY_DIR/slam_util.py"
  "$STAGING_DIR/ollama_api_util.py"
  "$STAGING_DIR/g1_academy_knowledge_de.json"
  "$STAGING_DIR/right_arm_forward_pose.json"
)
intros=(
  "$STAGING_DIR/first_task_day4_intro.html"
  "$STAGING_DIR/second_task_day4_intro.html"
  "$STAGING_DIR/third_task_day4_intro.html"
  "$STAGING_DIR/fourth_task_day4_intro.html"
  "$STAGING_DIR/fifth_task_day4_intro.html"
)
day_slides="$STAGING_DIR/day4_slides.html"

unsolved_pairs=(
  "$STAGING_DIR/first_task_day4_unsolved.ipynb::first_task_day4.ipynb"
  "$STAGING_DIR/second_task_day4_unsolved.ipynb::second_task_day4.ipynb"
  "$STAGING_DIR/third_task_day4_unsolved.ipynb::third_task_day4.ipynb"
  "$STAGING_DIR/fourth_task_day4_unsolved.ipynb::fourth_task_day4.ipynb"
  "$STAGING_DIR/fifth_task_day4_unsolved.ipynb::fifth_task_day4.ipynb"
)
solved_pairs=(
  "$STAGING_DIR/first_task_day4.ipynb::first_task_day4.ipynb"
  "$STAGING_DIR/second_task_day4.ipynb::second_task_day4.ipynb"
  "$STAGING_DIR/third_task_day4.ipynb::third_task_day4.ipynb"
  "$STAGING_DIR/fourth_task_day4.ipynb::fourth_task_day4.ipynb"
  "$STAGING_DIR/fifth_task_day4.ipynb::fifth_task_day4.ipynb"
)

# Retired Day 4 files (old OpenAI-based task12-16 naming, the brainco
# variant of task16, and the now-unused OpenAI util) that must NOT be
# staged; removed from the destination if present.
difficult_names=(
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
require_files "${deps[@]}" "${intros[@]}" "$day_slides"
for pair in "${unsolved_pairs[@]}" "${solved_pairs[@]}"; do
  require_files "${pair%%::*}"
done

restage_user() {
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
  cp "${intros[@]}" "$day_slides" "${deps[@]}" "$dest/"
  for name in "${difficult_names[@]}"; do
    rm -f "$dest/$name"
  done
  chown -R "$user:$user" "$dest"
  echo "restaged $user -> $dest"
}

for ((i=1; i<=NUM_USERS; i++)); do
  restage_user "${USER_PREFIX}${i}" "${unsolved_pairs[@]}"
done
restage_user "$SOLVED_USER" "${solved_pairs[@]}"

echo "Done. Restaged Day 4 (Ollama-based) for $NUM_USERS participant(s) + $SOLVED_USER (solved)."
echo "Reminder: participants must restart their Jupyter kernel, and need a running local Ollama server (ollama serve) with the chat/vision models pulled."
