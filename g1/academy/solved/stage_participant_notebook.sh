#!/usr/bin/env bash
set -euo pipefail

# Update ONE participant (unsolved) task notebook in each user's academy/day_N
# folder, without touching any other staged file. The notebook is copied
# verbatim from staging_day$DAY/ (the hand-authored participant version), so
# this is the surgical counterpart to stage_later_day_materials.sh -- use it
# when you have re-edited a single notebook and don't want to re-stage the
# whole day (intros, slides, sdk_wrapper.py, the other tasks).
#
# Usage:
#   sudo bash stage_participant_notebook.sh DAY TASK [USER ...]
# Examples:
#   sudo bash stage_participant_notebook.sh 2 6                 # -> teilnehmer1..12
#   sudo bash stage_participant_notebook.sh 2 6 teilnehmer3
#   sudo NUM_USERS=13 bash stage_participant_notebook.sh 2 6    # include teilnehmer13
#
# Default cohort is teilnehmer1..NUM_USERS (NUM_USERS=12: the unsolved
# participants; teilnehmer13 holds the solved copy). Override with an explicit
# USER list, or with NUM_USERS / USER_PREFIX.

day="${1:?Usage: sudo bash $0 DAY TASK [USER ...]}"
task="${2:?Usage: sudo bash $0 DAY TASK [USER ...]}"
shift 2 || true
users=("$@")

NUM_USERS="${NUM_USERS:-12}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
STAGING_DIR="$SOLVED_DIR/staging_day$day"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0 $day $task ${users[*]}" >&2; exit 1; }
[[ "$day" =~ ^[2-4]$ ]] || { echo "DAY must be 2, 3, or 4" >&2; exit 2; }
[[ "$task" =~ ^[0-9]+$ ]] || { echo "TASK must be a task number, e.g. 6" >&2; exit 2; }

# Resolve the single source notebook (there must be exactly one match).
matches=("$STAGING_DIR"/task"$task"_*.ipynb)
[[ -f "${matches[0]}" ]] || { echo "No notebook task${task}_*.ipynb in $STAGING_DIR" >&2; exit 1; }
(( ${#matches[@]} == 1 )) || { echo "Ambiguous: ${#matches[@]} notebooks match task${task}_*.ipynb in $STAGING_DIR" >&2; exit 1; }
notebook="${matches[0]}"
name="$(basename "$notebook")"

# Default to teilnehmer1..NUM_USERS when no explicit users were given.
if [[ ${#users[@]} -eq 0 ]]; then
  [[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
  [[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
  for ((i=1; i<=NUM_USERS; i++)); do users+=("${USER_PREFIX}${i}"); done
fi

for user in "${users[@]}"; do
  [[ "$user" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#user} <= 32 )) || { echo "Unsafe participant username: $user" >&2; exit 2; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  destination="$home_dir/academy/day_$day"
  install -d -m 0755 "$destination"
  rsync -a "$notebook" "$destination/"
  chown -- "$user:$user" "$destination/$name"
  echo "updated $user: $destination/$name"
done
echo "Staged $name to ${#users[@]} participant(s)."
