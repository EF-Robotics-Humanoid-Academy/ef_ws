#!/usr/bin/env bash
set -euo pipefail

# Grant academy users Ollama access and copy Unitree's aliases/API credential
# exports into each account. The requested API credentials are intentionally
# visible to every participant in their own ~/.academy-api.env.
# Usage: sudo bash grant_participant_tools.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
REF_BASHRC="${REF_BASHRC:-/home/unitree/.bashrc}"
REF_PY="${REF_PY:-/home/unitree/.guv/envs/unitree}"
REF_SDK="${REF_SDK:-/home/unitree/unitree_sdk2_python}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds}"
MARK_BEGIN='# >>> academy aliases and API environment (managed) >>>'
MARK_END='# <<< academy aliases and API environment (managed) <<<'

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
[[ -r "$REF_BASHRC" && -x "$REF_PY/bin/python" && -d "$REF_SDK/unitree_sdk2py" ]] || {
  echo "Reference shell configuration or Unitree runtime is unavailable." >&2; exit 1;
}
getent group ollama >/dev/null || { echo "The ollama group does not exist; install/start Ollama first." >&2; exit 1; }

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"

  # This contains only export statements whose variable names identify an API
  # credential; do not copy Unitree's interactive .bashrc startup code.
  api_env="$home_dir/.academy-api.env"
  grep -E '^[[:space:]]*export[[:space:]]+[A-Za-z_][A-Za-z0-9_]*(API_KEY|TOKEN|SECRET)[A-Za-z0-9_]*=' "$REF_BASHRC" >"$api_env" || true
  printf '%s\n' 'export OLLAMA_HOST="${OLLAMA_HOST:-http://127.0.0.1:11434}"' >>"$api_env"
  chown "$user:$user" "$api_env"
  chmod 0600 "$api_env"

  bashrc="$home_dir/.bashrc"
  touch "$bashrc"
  sed -i "\\|$MARK_BEGIN|,\\|$MARK_END|d" "$bashrc"
  {
    printf '\n%s\n' "$MARK_BEGIN"
    printf '%s\n' '[ -f "$HOME/.academy-api.env" ] && source "$HOME/.academy-api.env"'
    grep -E '^[[:space:]]*alias[[:space:]]+' "$REF_BASHRC" || true
    printf '%s\n' "$MARK_END"
  } >>"$bashrc"

  # Jupyter does not source ~/.bashrc.  Launch its Unitree kernel through a
  # small user-owned wrapper so the same API/Ollama environment is available
  # inside notebooks too.
  kernel_dir="$home_dir/.local/share/jupyter/kernels/unitree_sdk2"
  install -d -m 0755 "$kernel_dir"
  wrapper="$kernel_dir/launch-kernel.sh"
  cat >"$wrapper" <<EOF
#!/usr/bin/env bash
[ -f "\$HOME/.g1-unitree-env" ] && source "\$HOME/.g1-unitree-env"
source "\$HOME/.academy-api.env"
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
    "CYCLONEDDS_HOME": "$CYCLONEDDS_HOME",
    "CycloneDDS_ROOT": "$CYCLONEDDS_HOME",
    "LD_LIBRARY_PATH": "$CYCLONEDDS_HOME/lib",
    "PYTHONPATH": "$home_dir/unitree_sdk2_python",
    "OLLAMA_HOST": "http://127.0.0.1:11434"
  }
}
EOF
  chown -R "$user:$user" "$kernel_dir" "$bashrc"
  usermod -aG ollama "$user"
done

echo "Granted Ollama access and installed aliases/API environment for $NUM_USERS participants."
