#!/usr/bin/env bash
set -euo pipefail

# Create a self-contained Day 1 bundle in every participant's ~/academy.
# Usage: sudo bash stage_day1_materials.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
SOLVED_DIR="$(cd "$(dirname "$0")" && pwd)"
ACADEMY_DIR="$(cd "$SOLVED_DIR/.." && pwd)"
DAY1_SLIDES_SOURCE="$SOLVED_DIR/slides.html"
TEMPLATE_BUILDER="$SOLVED_DIR/make_day1_participant_templates.py"
REAL_IMAGES_DIR="$SOLVED_DIR/imgs_real"
# Per-task knowledge/reference pages (see staging_day1/) -- not part of the
# older solved/*.ipynb + slides.html pipeline above, staged separately here.
INTRO_DIR="$SOLVED_DIR/staging_day1"
INTRO_FILES=(
  "task1_sdkwrapper_usage_intro.html"
  "task2_necessary_dds_init_pubsub_intro.html"
  "task3_say_and_headlight_helpers_intro.html"
  "task4_robot_state_observation_intro.html"
)

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
for source in \
  "$SOLVED_DIR/task1_sdkwrapper_usage.ipynb" \
  "$SOLVED_DIR/task2_necessary_dds_init_pubsub.ipynb" \
  "$SOLVED_DIR/task3_say_and_headlight_helpers.ipynb" \
  "$SOLVED_DIR/task4_robot_state_observation.ipynb" \
  "$ACADEMY_DIR/sdk_wrapper.py" "$ACADEMY_DIR/util.py" "$DAY1_SLIDES_SOURCE" "$TEMPLATE_BUILDER"; do
  [[ -f "$source" ]] || { echo "Required Day 1 source is missing: $source" >&2; exit 1; }
done
[[ -d "$REAL_IMAGES_DIR" ]] || { echo "Required Day 1 image directory is missing: $REAL_IMAGES_DIR" >&2; exit 1; }
for intro_file in "${INTRO_FILES[@]}"; do
  [[ -f "$INTRO_DIR/$intro_file" ]] || { echo "Required Day 1 intro page is missing: $INTRO_DIR/$intro_file" >&2; exit 1; }
done

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  destination="$home_dir/academy/day_1"
  install -d -m 0755 "$destination"

  rsync -a \
    "$SOLVED_DIR/task1_sdkwrapper_usage.ipynb" \
    "$SOLVED_DIR/task2_necessary_dds_init_pubsub.ipynb" \
    "$SOLVED_DIR/task3_say_and_headlight_helpers.ipynb" \
    "$SOLVED_DIR/task4_robot_state_observation.ipynb" \
    "$ACADEMY_DIR/sdk_wrapper.py" "$ACADEMY_DIR/util.py" \
    "${INTRO_FILES[@]/#/$INTRO_DIR/}" \
    "$destination/"

  # Older bundles put a private 0700 imgs_real directory beside the deck.
  # The deck now uses the shared academy/docs location, so remove only that
  # obsolete, generated directory when refreshing the managed Day 1 bundle.
  rm -rf "$destination/imgs_real"

  docs_images="$home_dir/academy/docs/imgs"
  install -d -m 0755 "$docs_images"
  rsync -a "$REAL_IMAGES_DIR/" "$docs_images/"
  # The image source directory can be mode 0700. rsync -a preserves that
  # mode, which makes the files invisible to the participant/browser on this
  # root-squashed home mount. The screenshots are course material, so expose
  # them read-only to all academy accounts.
  chmod -R a+rX "$docs_images"

  # The checked-in notebooks are reference answers.  Produce participant
  # copies with imports, signatures, and safety constraints retained, but
  # the implementation lines replaced by explicit TODOs.
  python3 "$TEMPLATE_BUILDER" "$destination"

  # The source deck is a four-day deck. Retain slides s0 through s43, which
  # are the Day 1 introduction, Tasks 1-3 material, Day 1 recap, and close.
  python3 - "$DAY1_SLIDES_SOURCE" "$destination/day1_slides.html" "$REAL_IMAGES_DIR" <<'PY'
import base64
import mimetypes
import re
import sys
from pathlib import Path

source = Path(sys.argv[1]).read_text(encoding="utf-8")
image_dir = Path(sys.argv[3])
head = re.search(r'(?s)\A(.*?</head>)', source)
sections = re.findall(r'(?s)<section class="slide.*?</section>', source)
day_one = sections[:44]
if head is None or len(day_one) != 44:
    raise SystemExit("Could not extract the expected 44 Day 1 slides")

deck = "\n".join(day_one)
# The source deck's script swaps placeholders for relative files at runtime.
# Participant decks instead embed those images below, so retaining that script
# would overwrite data URLs and trigger JupyterHub's sandbox-origin 403.
head_html = re.sub(
    r'(?s)<script>\s*// Replace illustrative placeholders with the cohort.*?</script>\s*',
    '',
    head.group(1),
)
page = f'''{head_html}
<body>
<div class="deck">{deck}</div>
<div class="controls">
  <button id="btn-prev" title="Previous">&#8592;</button>
  <span class="counter" id="counter"></span>
  <button id="btn-next" title="Next">&#8594;</button>
</div>
<script>
(() => {{
  const slides = Array.from(document.querySelectorAll('.slide'));
  let index = 0;
  const render = () => {{
    slides.forEach((slide, i) => slide.classList.toggle('active', i === index));
    document.getElementById('counter').textContent = `${{index + 1}} / ${{slides.length}}`;
    history.replaceState(null, '', `#s${{index}}`);
  }};
  const go = (next) => {{ index = Math.max(0, Math.min(slides.length - 1, next)); render(); }};
  document.getElementById('btn-prev').onclick = () => go(index - 1);
  document.getElementById('btn-next').onclick = () => go(index + 1);
  window.addEventListener('keydown', (event) => {{
    if (['ArrowRight', ' ', 'PageDown'].includes(event.key)) {{ go(index + 1); event.preventDefault(); }}
    if (['ArrowLeft', 'PageUp'].includes(event.key)) {{ go(index - 1); event.preventDefault(); }}
  }});
  const match = location.hash.match(/^#s(\\d+)$/);
  if (match) index = Math.min(slides.length - 1, Number(match[1]));
  render();
}})();
</script>
</body>
</html>
'''
# JupyterHub's HTML preview may refuse inline JavaScript.  Bake the shared
# academy image URLs directly into the generated Day 1 deck instead of
# relying on the source deck's browser-side image replacement script.
real_images = {
    'App main view — home screen with robot status, mode indicator, and battery.': 'unitree_app_main.png',
    "App connection screen — pairing/joining the robot's network or the 192.168.123.0/24 subnet.": 'unitree_app_connection.jpeg',
    'App settings screen — network, volume, firmware/version info.': 'unitree_app_settings.jpeg',
    'App control screen — joystick-style walk control and mode buttons.': 'unitree_app_control.jpeg',
    'App debugging/diagnostics screen — service status, logs, and fault codes.': 'unitree_app_debugging.jpeg',
    'Livox Mid-360 LiDAR — mounted position on the G1 and its 360-degree point-cloud coverage pattern.': 'unitree_app_slam.jpeg',
    'Dex3 hand — 3-finger dexterous hand with its 7 joints and tactile pads called out.': 'dex3_hand.jpg',
    'Inspire hand — 5-finger hand mounted on the wrist, with the network-cable route highlighted.': 'inspire_5_finger_hand.jpg',
}
for alt, filename in real_images.items():
    pattern = rf'(<img\s+src=")[^"]+(" alt="{re.escape(alt)}")'
    image_path = image_dir / filename
    if not image_path.is_file():
        raise SystemExit(f"Missing staged image: {image_path}")
    mime_type = mimetypes.guess_type(filename)[0] or 'application/octet-stream'
    data_url = f"data:{mime_type};base64,{base64.b64encode(image_path.read_bytes()).decode('ascii')}"
    # Embed the cohort images so JupyterHub's HTML preview never has to
    # resolve a sibling/parent file URL (which it commonly blocks).
    page, count = re.subn(pattern, rf'\1{data_url}\2', page)
    if count != 1:
        raise SystemExit(f"Could not replace staged image for: {alt}")
for alt in (
    'A real G1 unit on a test stand — the actual hardware behind every diagram on this slide.',
    'A real `ros2 topic list` terminal session — this is what the note below should show once ROS 2 is actually reachable.',
    'An example CLI session — one of this deck\'s suggested prompts pasted in, and the CLI implementing it.',
):
    pattern = rf'<div class="figure real-image"><img\s+src="[^"]+" alt="{re.escape(alt)}"[^>]*>.*?</div></div>'
    page, count = re.subn(pattern, '', page, flags=re.S)
    if count != 1:
        raise SystemExit(f"Could not remove staged figure for: {alt}")
Path(sys.argv[2]).write_text(page, encoding="utf-8")
PY
  chown -R "$user:$user" "$destination" "$docs_images"
done

echo "Created ~/academy/day_1 bundles for $NUM_USERS participants."
