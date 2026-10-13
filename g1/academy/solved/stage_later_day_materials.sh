#!/usr/bin/env bash
set -euo pipefail

# Stage a self-contained participant bundle for Day 2, 3, or 4.
# Usage: sudo bash stage_later_day_materials.sh DAY_NUMBER
# Day 3 Task 11 defaults to Dex3; set HAND_VARIANT=brainco for Revo 2.

day="${1:?Usage: sudo bash $0 DAY_NUMBER (2, 3, or 4)}"
NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
HAND_VARIANT="${HAND_VARIANT:-dex3}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0 $day" >&2; exit 1; }
case "$day" in
  2) tasks=(5 6 7); first_slide=44; last_slide=65 ;;
  3) tasks=(8 9 10 11); first_slide=66; last_slide=95 ;;
  4) tasks=(first second third fourth fifth); first_slide=0; last_slide=0 ;;
  *) echo "DAY_NUMBER must be 2, 3, or 4" >&2; exit 2 ;;
esac
if [[ ! "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] || (( NUM_USERS > 100 )); then
  echo "NUM_USERS must be 1..100" >&2
  exit 2
fi
if [[ ! "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] || (( ${#USER_PREFIX} > 24 )); then
  echo "USER_PREFIX must be a non-empty, safe Linux username prefix (max 24 characters)" >&2
  exit 2
fi
[[ "$HAND_VARIANT" == "dex3" || "$HAND_VARIANT" == "brainco" ]] || {
  echo "HAND_VARIANT must be 'dex3' or 'brainco'" >&2
  exit 2
}
for required_command in getent install rsync python3 cp chown; do
  command -v "$required_command" >/dev/null || {
    echo "Required command is not installed: $required_command" >&2
    exit 1
  }
done

# Per-task knowledge/reference pages (see staging_dayN/) -- not part of the
# solved/*.ipynb + slides.html pipeline above, staged separately here.
STAGING_DIR="$SOLVED_DIR/staging_day$day"

# Day 4 was migrated from the retired OpenAI task12-task16 bundle to the
# current Ollama first..fifth bundle. Keep this helper aligned with
# restage_day4.sh so it cannot reintroduce the old notebooks or API helper.
if [[ "$day" == 4 ]]; then
  notebooks=(
    "$STAGING_DIR/first_task_day4_unsolved.ipynb"
    "$STAGING_DIR/second_task_day4_unsolved.ipynb"
    "$STAGING_DIR/third_task_day4_unsolved.ipynb"
    "$STAGING_DIR/fourth_task_day4_unsolved.ipynb"
    "$STAGING_DIR/fifth_task_day4_unsolved.ipynb"
  )
  intros=(
    "$STAGING_DIR/first_task_day4_intro.html"
    "$STAGING_DIR/second_task_day4_intro.html"
    "$STAGING_DIR/third_task_day4_intro.html"
    "$STAGING_DIR/fourth_task_day4_intro.html"
    "$STAGING_DIR/fifth_task_day4_intro.html"
  )
  shared=(
    "$STAGING_DIR/day4_slides.html"
    "$ACADEMY_DIR/sdk_wrapper.py"
    "$ACADEMY_DIR/util.py"
    "$ACADEMY_DIR/slam_util.py"
    "$ACADEMY_DIR/ollama_api_util.py"
    "$STAGING_DIR/g1_academy_knowledge_de.json"
    "$STAGING_DIR/right_arm_forward_pose.json"
  )
  for source in "${notebooks[@]}" "${intros[@]}" "${shared[@]}"; do
    [[ -f "$source" ]] || { echo "Missing Day 4 source: $source" >&2; exit 1; }
  done
  for ((i=1; i<=NUM_USERS; i++)); do
    user="${USER_PREFIX}${i}"
    home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
    [[ -n "$home_dir" && "$home_dir" == /* && "$home_dir" != / ]] || {
      echo "Missing or unsafe home directory for $user" >&2; exit 1;
    }
    destination="$home_dir/academy/day_4"
    install -d -m 0755 "$destination"
    cp -- "${notebooks[0]}" "$destination/first_task_day4.ipynb"
    cp -- "${notebooks[1]}" "$destination/second_task_day4.ipynb"
    cp -- "${notebooks[2]}" "$destination/third_task_day4.ipynb"
    cp -- "${notebooks[3]}" "$destination/fourth_task_day4.ipynb"
    cp -- "${notebooks[4]}" "$destination/fifth_task_day4.ipynb"
    rsync -a "${intros[@]}" "${shared[@]}" "$destination/"
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
    chown -R -- "$user:$user" "$destination"
    echo "Staged current Ollama Day 4 materials for $user."
  done
  exit 0
fi

# Some days ship hand-authored, participant-ready notebooks in staging_day$day/
# (the setup cell is intact and only the answer cells are stubbed). For those
# days, copy the staged notebook verbatim; every other day falls back to
# stripping the solved/ reference notebook into an exercise (see below).
# VERBATIM_DAYS lists which days are already hand-authored this way.
VERBATIM_DAYS="${VERBATIM_DAYS:-2}"
verbatim_day=0
for d in $VERBATIM_DAYS; do [[ "$d" == "$day" ]] && verbatim_day=1; done

strip_notebooks=()    # solved/ references to strip into exercises at the destination
verbatim_notebooks=() # staging_day$day/ participant notebooks copied as-is
for task in "${tasks[@]}"; do
  if [[ "$task" == 11 ]]; then
    staged=("$STAGING_DIR"/task11_"$HAND_VARIANT"_*.ipynb)
    solved_nb=("$SOLVED_DIR"/task11_"$HAND_VARIANT"_*.ipynb)
  else
    staged=("$STAGING_DIR"/task"$task"_*_unsolved.ipynb)
    [[ -f "${staged[0]}" ]] || staged=("$STAGING_DIR"/task"$task"_*.ipynb)
    solved_nb=("$SOLVED_DIR"/task"$task"_*.ipynb)
  fi
  if (( verbatim_day )) && [[ -f "${staged[0]}" ]]; then
    (( ${#staged[@]} == 1 )) || {
      echo "Ambiguous Task $task participant notebooks (${#staged[@]} matches)" >&2
      exit 1
    }
    verbatim_notebooks+=("${staged[0]}")
  elif [[ -f "${solved_nb[0]}" ]]; then
    (( ${#solved_nb[@]} == 1 )) || {
      echo "Ambiguous Task $task solved notebooks (${#solved_nb[@]} matches)" >&2
      exit 1
    }
    strip_notebooks+=("${solved_nb[0]}")
  else
    echo "Missing Task $task notebook (looked in $STAGING_DIR and $SOLVED_DIR)" >&2; exit 1
  fi
done

intros=()
for task in "${tasks[@]}"; do
  if [[ "$task" == 11 ]]; then
    match=("$STAGING_DIR"/task11_"$HAND_VARIANT"_*_intro.html)
  else
    match=("$STAGING_DIR"/task"$task"_*_intro.html)
  fi
  [[ -f "${match[0]}" ]] || { echo "Missing Task $task intro page in $STAGING_DIR" >&2; exit 1; }
  (( ${#match[@]} == 1 )) || { echo "Ambiguous Task $task intro pages (${#match[@]} matches)" >&2; exit 1; }
  intros+=("${match[0]}")
done

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  home_dir="$(getent passwd "$user" | cut -d: -f6 || true)"
  [[ -n "$home_dir" ]] || { echo "Missing account: $user" >&2; exit 1; }
  destination="$home_dir/academy/day_$day"
  install -d -m 0755 "$destination"
  rsync -a \
    ${strip_notebooks[@]+"${strip_notebooks[@]}"} \
    ${verbatim_notebooks[@]+"${verbatim_notebooks[@]}"} \
    "${intros[@]}" \
    "$ACADEMY_DIR/sdk_wrapper.py" "$ACADEMY_DIR/util.py" "$ACADEMY_DIR/slam_util.py" \
    "$destination/"

  # Notebooks copied from staging_day$day/ are already participant-ready and are
  # left untouched. Notebooks taken from solved/ are reference answers: strip
  # their solved code cells into TODO exercises, preserving the task prose but
  # not distributing solutions. Imports stay visible as starting clues.
  if (( ${#strip_notebooks[@]} )); then
  python3 - "$destination" "${strip_notebooks[@]}" <<'PY'
import json, re, sys
from pathlib import Path
dest = Path(sys.argv[1])
for source in map(Path, sys.argv[2:]):
    path = dest / source.name
    nb = json.loads(path.read_text(encoding='utf-8'))
    for cell in nb['cells']:
        if cell.get('cell_type') != 'code':
            continue
        original = ''.join(cell.get('source', []))
        imports = [line for line in original.splitlines() if re.match(r'^(?:from\s+\S+\s+import\s+|import\s+)', line)]
        cell['source'] = imports + [
            '', '# TODO: Implement this section using the preceding task description and day slides.',
            '# Keep robot commands conservative; test observation/state code before actuation.',
            'raise NotImplementedError("Complete this task section")',
        ]
        cell['source'] = [line + '\n' for line in cell['source']]
        cell['outputs'] = []
        cell['execution_count'] = None
    path.write_text(json.dumps(nb, indent=1, ensure_ascii=False) + '\n', encoding='utf-8')
PY
  fi

  python3 - "$SOLVED_DIR/slides.html" "$destination/day${day}_slides.html" "$first_slide" "$last_slide" <<'PY'
import re, sys
from pathlib import Path
source = Path(sys.argv[1]).read_text(encoding='utf-8')
head = re.search(r'(?s)\A(.*?</head>)', source)
sections = re.findall(r'(?s)<section class="slide.*?</section>', source)
first, last = int(sys.argv[3]), int(sys.argv[4])
if not head or len(sections) <= last: raise SystemExit('Could not extract day slides')
deck = '\n'.join(sections[first:last+1])
page = '''%s
<body><div class="deck">%s</div>
<div class="controls"><button id="btn-prev">&#8592;</button><span class="counter" id="counter"></span><button id="btn-next">&#8594;</button></div>
<script>
(() => {
  const slides = Array.from(document.querySelectorAll('.slide')); let index = 0;
  const render = () => { slides.forEach((s, i) => s.classList.toggle('active', i === index)); document.getElementById('counter').textContent = `${index + 1} / ${slides.length}`; };
  document.getElementById('btn-prev').onclick = () => { index = Math.max(0, index - 1); render(); };
  document.getElementById('btn-next').onclick = () => { index = Math.min(slides.length - 1, index + 1); render(); };
  window.addEventListener('keydown', e => { if (['ArrowRight', ' ', 'PageDown'].includes(e.key)) { index = Math.min(slides.length - 1, index + 1); render(); e.preventDefault(); } if (['ArrowLeft', 'PageUp'].includes(e.key)) { index = Math.max(0, index - 1); render(); e.preventDefault(); } });
  render();
})();
</script></body></html>
''' % (head.group(1), deck)
Path(sys.argv[2]).write_text(page, encoding='utf-8')
PY
  chown -R "$user:$user" "$destination"
done
echo "Staged Day $day materials for $NUM_USERS participants."
