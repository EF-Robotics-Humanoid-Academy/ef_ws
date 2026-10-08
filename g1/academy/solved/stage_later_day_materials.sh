#!/usr/bin/env bash
set -euo pipefail

# Stage a self-contained participant bundle for Day 2, 3, or 4.
# Usage: sudo bash stage_later_day_materials.sh DAY_NUMBER

day="${1:?Usage: sudo bash $0 DAY_NUMBER (2, 3, or 4)}"
NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0 $day" >&2; exit 1; }
case "$day" in
  2) tasks=(5 6 7); first_slide=44; last_slide=65 ;;
  3) tasks=(8 9 10 11); first_slide=66; last_slide=95 ;;
  4) tasks=(12 13); first_slide=96; last_slide=109 ;;
  *) echo "DAY_NUMBER must be 2, 3, or 4" >&2; exit 2 ;;
esac
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || {
  echo "NUM_USERS must be 1..100" >&2
  exit 2
}
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || {
  echo "USER_PREFIX must be a non-empty, safe Linux username prefix (max 24 characters)" >&2
  exit 2
}
for required_command in getent install rsync python3 chown; do
  command -v "$required_command" >/dev/null || {
    echo "Required command is not installed: $required_command" >&2
    exit 1
  }
done

# Per-task knowledge/reference pages (see staging_dayN/) -- not part of the
# solved/*.ipynb + slides.html pipeline above, staged separately here.
STAGING_DIR="$SOLVED_DIR/staging_day$day"

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
  staged=("$STAGING_DIR"/task"$task"_*.ipynb)
  solved_nb=("$SOLVED_DIR"/task"$task"_*.ipynb)
  if (( verbatim_day )) && [[ -f "${staged[0]}" ]]; then
    verbatim_notebooks+=("${staged[0]}")
  elif [[ -f "${solved_nb[0]}" ]]; then
    strip_notebooks+=("${solved_nb[0]}")
  else
    echo "Missing Task $task notebook (looked in $STAGING_DIR and $SOLVED_DIR)" >&2; exit 1
  fi
done

intros=()
for task in "${tasks[@]}"; do
  match=("$STAGING_DIR"/task"$task"_*_intro.html)
  [[ -f "${match[0]}" ]] || { echo "Missing Task $task intro page in $STAGING_DIR" >&2; exit 1; }
  intros+=("${match[0]}")
done

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
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
