#!/usr/bin/env bash
set -euo pipefail

# Install the runtime tools used by academy notebooks once in the tested
# Unitree venv, then expose them read-only to every participant.
# Usage: sudo bash install_academy_notebook_tools.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
REF_USER="${REF_USER:-unitree}"
REF_PY="${REF_PY:-/home/unitree/.guv/envs/unitree}"
OPENAIAPI_DIR="${OPENAIAPI_DIR:-/home/unitree/EF/ef_ws/g1/modules/scripts/openaiapi}"
REF_VOICES="${REF_VOICES:-/home/unitree/.local/share/piper/voices}"
SHARED_ROOT="${SHARED_ROOT:-/opt/academy-tools}"
VOICE_DIR="$SHARED_ROOT/piper-voices"
MARK_BEGIN='# >>> academy notebook tools (managed) >>>'
MARK_END='# <<< academy notebook tools (managed) <<<'

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
[[ -x "$REF_PY/bin/python" && -d "$OPENAIAPI_DIR/agent" && -d "$REF_VOICES" ]] || {
  echo "Unitree Python runtime, openaiapi source, or Piper voices are missing." >&2; exit 1;
}

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y espeak-ng

# Keep ownership with the reference account, not root, because this is its
# existing virtual environment.  These are the optional tools academy
# notebooks and openaiapi use beyond the SDK/DDS stack.
runuser -u "$REF_USER" -- "$REF_PY/bin/python" -m pip install \
  openai piper-tts

install -d -m 0755 "$VOICE_DIR"
rsync -a "$REF_VOICES/" "$VOICE_DIR/"
chmod -R a+rX "$VOICE_DIR"

"$REF_PY/bin/python" - <<'PY'
import cyclonedds, cv2, numpy, openai, pandas, pydantic, unitree_sdk2py
print("academy Python imports: OK")
PY
"$REF_PY/bin/piper" --help >/dev/null

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"

  tools_env="$home_dir/.academy-tools-env"
  cat >"$tools_env" <<EOF
# Managed by install_academy_notebook_tools.sh
export G1_OPENAIAPI_DIR="$OPENAIAPI_DIR"
export G1_PIPER_BIN="$REF_PY/bin/piper"
export G1_PIPER_VOICE_DIR="$VOICE_DIR"
export PYTHONPATH="\$G1_OPENAIAPI_DIR\${PYTHONPATH:+:\$PYTHONPATH}"
alias openaiapi='cd "\$G1_OPENAIAPI_DIR" && python -m agent.cli'
EOF
  chown "$user:$user" "$tools_env"
  chmod 0600 "$tools_env"

  # The academy wrapper looks up voices under the standard Piper user path.
  # Expose the single shared, read-only model there rather than copying it.
  piper_home="$home_dir/.local/share/piper"
  voice_path="$piper_home/voices"
  install -d -m 0755 "$piper_home"
  if [[ -L "$voice_path" ]]; then
    ln -sfn "$VOICE_DIR" "$voice_path"
    chown -h "$user:$user" "$voice_path"
  elif [[ -d "$voice_path" ]]; then
    cp -a "$VOICE_DIR/." "$voice_path/"
    chown -R "$user:$user" "$voice_path"
  elif [[ -e "$voice_path" ]]; then
    echo "Cannot configure Piper for $user: $voice_path is not a directory or symlink." >&2
    exit 1
  else
    ln -s "$VOICE_DIR" "$voice_path"
    chown -h "$user:$user" "$voice_path"
  fi
  chown "$user:$user" "$piper_home"

  bashrc="$home_dir/.bashrc"
  touch "$bashrc"
  sed -i "\\|$MARK_BEGIN|,\\|$MARK_END|d" "$bashrc"
  {
    printf '\n%s\n' "$MARK_BEGIN"
    printf '%s\n' '[ -f "$HOME/.academy-tools-env" ] && source "$HOME/.academy-tools-env"'
    printf '%s\n' "$MARK_END"
  } >>"$bashrc"

  # Preserve the Unitree SDK/DDS kernel, adding notebook-only academy tools.
  kernel_dir="$home_dir/.local/share/jupyter/kernels/unitree_sdk2"
  install -d -m 0755 "$kernel_dir"
  wrapper="$kernel_dir/launch-kernel.sh"
  cat >"$wrapper" <<EOF
#!/usr/bin/env bash
[ -f "\$HOME/.academy-api.env" ] && source "\$HOME/.academy-api.env"
[ -f "\$HOME/.academy-tools-env" ] && source "\$HOME/.academy-tools-env"
exec "$REF_PY/bin/python" -m ipykernel_launcher "\$@"
EOF
  chmod 0700 "$wrapper"
  cat >"$kernel_dir/kernel.json" <<EOF
{
  "argv": ["$wrapper", "-f", "{connection_file}"],
  "display_name": "Unitree SDK2 (g1 academy)",
  "language": "python",
  "env": {
    "UNITREE_SDK2_HOME": "$home_dir/unitree_sdk2_python",
    "CYCLONEDDS_HOME": "/home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds",
    "CycloneDDS_ROOT": "/home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds",
    "LD_LIBRARY_PATH": "/home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds/lib",
    "PYTHONPATH": "$home_dir/unitree_sdk2_python:$OPENAIAPI_DIR",
    "G1_OPENAIAPI_DIR": "$OPENAIAPI_DIR",
    "G1_PIPER_BIN": "$REF_PY/bin/piper",
    "G1_PIPER_VOICE_DIR": "$VOICE_DIR",
    "OLLAMA_HOST": "http://127.0.0.1:11434"
  }
}
EOF
  chown -R "$user:$user" "$kernel_dir" "$bashrc"
done

echo "Installed academy notebook tools and shared Piper voices for $NUM_USERS participants."
