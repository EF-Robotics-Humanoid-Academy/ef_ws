#!/usr/bin/env bash
set -euo pipefail

# reset_users.sh — prune and recreate the academy participant accounts
# (teilnehmer1..teilnehmer13) and give every one of them a working copy of
# the Python cyclonedds + unitree_sdk2_python setup, cloned from the
# reference "unitree" account so the paths and environment variables match
# exactly. Also makes sure every account can reliably use: Piper
# text-to-speech, the openaiapi agent, IPython/Jupyter, rich, pandas,
# numpy, opencv, and password SSH login.
#
# - Participant accounts get NO sudo rights.
# - Every participant account gets the same SSH/login password supplied via
#   TEILNEHMER_PASSWORD. It is never hard-coded in this script.
# - Existing teilnehmerN accounts (and their home directories) are deleted
#   first, then recreated from scratch. This is destructive: review before
#   running on a live classroom machine.
#
# Usage: sudo ./reset_users.sh --yes

usage() {
  cat <<'EOF'
Usage: sudo reset_users.sh --yes

Deletes and recreates academy participant accounts. Set configuration through
NUM_USERS, USER_PREFIX, REF_USER, SHELL_BIN, and TEILNEHMER_PASSWORD.
The --yes flag (or RESET_USERS_CONFIRM=YES) is required because this removes
existing accounts and home directories.
EOF
}

ASSUME_YES=0
while (($#)); do
  case "$1" in
    --yes) ASSUME_YES=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
  shift
done

if [[ "$(id -u)" -ne 0 ]]; then
  echo "This script must be run as root (e.g. sudo $0)." >&2
  exit 1
fi
if [[ "$ASSUME_YES" -ne 1 && "${RESET_USERS_CONFIRM:-}" != "YES" ]]; then
  echo "Refusing destructive account reset without --yes or RESET_USERS_CONFIRM=YES." >&2
  exit 1
fi

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
REF_USER="${REF_USER:-unitree}"
SHELL_BIN="${SHELL_BIN:-/bin/bash}"
TEILNEHMER_PASSWORD="${TEILNEHMER_PASSWORD:-}"
# Shared, world-readable cache so all 13 accounts see the same Piper voice
# models without each needing its own copy under ~/.local/share/piper.
SHARED_VOICE_DIR="${SHARED_VOICE_DIR:-/opt/piper-voices}"
# Matches util.py's PIPER_VOICES map — keep these in sync.
PIPER_VOICES=(en_US-lessac-medium de_DE-thorsten-medium fr_FR-siwis-medium es_ES-davefx-medium)

if [[ ! "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] || ((NUM_USERS > 100)); then
  echo "NUM_USERS must be an integer between 1 and 100." >&2
  exit 1
fi
if [[ ! "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] || ((${#USER_PREFIX} > 24)); then
  echo "USER_PREFIX must be a safe lowercase Linux account prefix (maximum 24 characters)." >&2
  exit 1
fi
if [[ ! "$REF_USER" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then
  echo "REF_USER is not a valid Linux account name." >&2
  exit 1
fi
if [[ ! -x "$SHELL_BIN" ]]; then
  echo "SHELL_BIN is not executable: $SHELL_BIN" >&2
  exit 1
fi
if [[ -z "$TEILNEHMER_PASSWORD" && -t 0 ]]; then
  read -r -s -p "Password for all participant accounts: " TEILNEHMER_PASSWORD
  printf '\n'
fi
if [[ -z "$TEILNEHMER_PASSWORD" ]]; then
  echo "Set TEILNEHMER_PASSWORD or run this script from a terminal to enter it." >&2
  exit 1
fi
if [[ "$TEILNEHMER_PASSWORD" == *:* || "$TEILNEHMER_PASSWORD" == *$'\n'* || "$TEILNEHMER_PASSWORD" == *$'\r'* ]]; then
  echo "TEILNEHMER_PASSWORD must not contain a colon or newline." >&2
  exit 1
fi
trap 'unset TEILNEHMER_PASSWORD' EXIT

for command_name in getent useradd userdel chpasswd rsync find sed grep; do
  command -v "$command_name" >/dev/null 2>&1 || {
    echo "Required command not found: $command_name" >&2
    exit 1
  }
done

USERS=()
for ((i = 1; i <= NUM_USERS; i++)); do
  USERS+=("${USER_PREFIX}${i}")
done
for u in "${USERS[@]}"; do
  if [[ "$u" == "$REF_USER" ]]; then
    echo "REF_USER must not be one of the participant accounts scheduled for deletion: $u" >&2
    exit 1
  fi
done

# --- SSH: make sure password login actually works -----------------------
ensure_ssh_password_auth() {
  local cfg="/etc/ssh/sshd_config" changed=0
  if [[ ! -f "$cfg" ]]; then
    echo "note: $cfg not found; is openssh-server installed? skipping sshd check" >&2
    return
  fi
  if grep -qiE '^\s*PasswordAuthentication\s+no' "$cfg"; then
    sed -i 's/^\s*PasswordAuthentication\s\+no/PasswordAuthentication yes/I' "$cfg"
    changed=1
  elif ! grep -qiE '^\s*PasswordAuthentication\s+yes' "$cfg"; then
    echo "PasswordAuthentication yes" >> "$cfg"
    changed=1
  fi
  # A drop-in under sshd_config.d overrides the main file on modern distros.
  for override in /etc/ssh/sshd_config.d/*.conf; do
    [[ -f "$override" ]] || continue
    if grep -qiE '^\s*PasswordAuthentication\s+no' "$override"; then
      sed -i 's/^\s*PasswordAuthentication\s\+no/PasswordAuthentication yes/I' "$override"
      changed=1
    fi
  done
  if [[ "$changed" -eq 1 ]]; then
    systemctl reload sshd 2>/dev/null || systemctl reload ssh 2>/dev/null || service ssh reload 2>/dev/null || true
    echo "Enabled PasswordAuthentication in $cfg and reloaded sshd."
  fi
}

REF_HOME="$(getent passwd "$REF_USER" | cut -d: -f6 || true)"
if [[ -z "$REF_HOME" || ! -d "$REF_HOME" ]]; then
  echo "Reference user '$REF_USER' not found or has no home directory; cannot clone its setup." >&2
  exit 1
fi

# --- Locate the reference workspace -----------------------------------
# Detect the directory that actually holds a built native CycloneDDS
# install plus the unitree_sdk2_python checkout, instead of hardcoding a
# path, so this keeps working even if the reference machine's layout
# differs from the documented default ($HOME/unitree_sdk2_ws).
find_ref_workspace() {
  local libddsc prefix candidate
  while IFS= read -r -d '' libddsc; do
    prefix="$(dirname "$(dirname "$libddsc")")"
    for candidate in "$(dirname "$prefix")" "$(dirname "$(dirname "$prefix")")"; do
      if [[ -d "$candidate/unitree_sdk2_python" ]]; then
        echo "$candidate"
        return 0
      fi
    done
  done < <(find "$REF_HOME" -maxdepth 7 -name 'libddsc.so*' -path '*/lib/*' -print0 2>/dev/null)
  return 1
}

REF_WS="$(find_ref_workspace || true)"
if [[ -z "$REF_WS" ]]; then
  REF_WS="$REF_HOME/unitree_sdk2_ws"
fi
if [[ ! -d "$REF_WS" ]]; then
  echo "Could not locate a built unitree_sdk2_python/cyclonedds workspace under $REF_HOME." >&2
  exit 1
fi
if [[ "$REF_WS" != "$REF_HOME/"* ]]; then
  echo "Reference workspace must be inside $REF_HOME; refusing unsafe path: $REF_WS" >&2
  exit 1
fi
if [[ ! -d "$REF_WS/unitree_sdk2_python" ]]; then
  echo "Reference workspace has no unitree_sdk2_python checkout: $REF_WS" >&2
  exit 1
fi
REF_WS_REL="${REF_WS#"$REF_HOME"/}"
echo "Reference workspace: $REF_WS (will be cloned to \$HOME/$REF_WS_REL for each participant)"

REF_CYCLONE_PREFIX=""
while IFS= read -r -d '' libddsc; do
  REF_CYCLONE_PREFIX="$(dirname "$(dirname "$libddsc")")"
  break
done < <(find "$REF_WS" -maxdepth 6 -name 'libddsc.so*' -path '*/lib/*' -print0 2>/dev/null)
if [[ -z "$REF_CYCLONE_PREFIX" ]]; then
  echo "Could not locate the CycloneDDS install prefix under $REF_WS." >&2
  exit 1
fi
REF_CYCLONE_REL="${REF_CYCLONE_PREFIX#"$REF_WS"/}"

REF_VENV_REL=""
while IFS= read -r -d '' activate; do
  REF_VENV_REL="${activate%/bin/activate}"
  REF_VENV_REL="${REF_VENV_REL#"$REF_WS"/}"
  break
done < <(find "$REF_WS" -maxdepth 4 -type f -name activate -path '*/bin/activate' -print0 2>/dev/null)

# All read-only preflight checks succeeded. Apply the requested global SSH
# configuration only now, before account creation begins.
ensure_ssh_password_auth

# --- Shared Piper TTS voice models --------------------------------------
# util.py/sdk_wrapper.py resolve voices under $G1_PIPER_VOICE_DIR (default
# ~/.local/share/piper/voices — per-user, and empty on a fresh account).
# Seed one shared copy from whatever unitree already has cached, topping up
# anything missing from Hugging Face, so TTS works out of the box for
# every participant without downloading it 13 times.
mkdir -p "$SHARED_VOICE_DIR"
for cand in "$REF_HOME/.local/share/piper/voices" "$REF_WS/piper-voices" "$REF_HOME/piper-voices"; do
  [[ -d "$cand" ]] && rsync -a --ignore-existing "$cand/" "$SHARED_VOICE_DIR/" 2>/dev/null
done
for voice in "${PIPER_VOICES[@]}"; do
  locale="${voice%%-*}"; rest="${voice#*-}"; name="${rest%-*}"; quality="${rest##*-}"; lang="${locale%%_*}"
  dest="$SHARED_VOICE_DIR/$voice"
  mkdir -p "$dest"
  base="https://huggingface.co/rhasspy/piper-voices/resolve/main/$lang/$locale/$name/$quality"
  for ext in onnx onnx.json; do
    [[ -f "$dest/$voice.$ext" ]] || curl -fsSL "$base/$voice.$ext" -o "$dest/$voice.$ext" 2>/dev/null \
      || echo "warning: could not fetch Piper voice $voice.$ext (check network)" >&2
  done
done
chmod -R a+rX "$SHARED_VOICE_DIR"
echo "Piper voices available at $SHARED_VOICE_DIR"

# --- Extract the reference environment block ---------------------------
# Pull the actual exports out of unitree's shell rc so every participant
# gets byte-for-byte the same CYCLONEDDS_HOME / CycloneDDS_ROOT / PATH /
# CMAKE_PREFIX_PATH / LD_LIBRARY_PATH / PKG_CONFIG_PATH (and any ROS
# sourcing) configuration, rather than a guessed reconstruction.
REF_RC=""
for candidate in "$REF_HOME/.bashrc" "$REF_HOME/.profile"; do
  if [[ -f "$candidate" ]] && grep -q "CYCLONEDDS_HOME\|unitree_sdk2" "$candidate"; then
    REF_RC="$candidate"
    break
  fi
done

if [[ -n "$REF_RC" ]]; then
  ENV_BLOCK="$(grep -E \
    'CYCLONEDDS_HOME|CycloneDDS_ROOT|UNITREE_WS|LD_LIBRARY_PATH|CMAKE_PREFIX_PATH|PKG_CONFIG_PATH|PYTHONPATH|unitree_sdk2|ros/[A-Za-z0-9_]+/setup\.bash|/?\.?venv/bin/activate|CYCLONEDDS' \
    "$REF_RC" || true)"
  echo "Cloned environment block from $REF_RC."
fi
if [[ -z "${ENV_BLOCK:-}" ]]; then
  echo "No CycloneDDS/unitree_sdk2 environment block found in $REF_USER's rc files; generating the documented default." >&2
  ENV_BLOCK=$(cat <<EOF
export UNITREE_WS="\$HOME/$REF_WS_REL"
export CYCLONEDDS_HOME="\$UNITREE_WS/$REF_CYCLONE_REL"
export CycloneDDS_ROOT="\$CYCLONEDDS_HOME"
export PATH="\$CYCLONEDDS_HOME/bin\${PATH:+:\$PATH}"
export CMAKE_PREFIX_PATH="\$CYCLONEDDS_HOME\${CMAKE_PREFIX_PATH:+:\$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="\$CYCLONEDDS_HOME/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export PKG_CONFIG_PATH="\$CYCLONEDDS_HOME/lib/pkgconfig\${PKG_CONFIG_PATH:+:\$PKG_CONFIG_PATH}"
export PYTHONPATH="\$UNITREE_WS/unitree_sdk2_python\${PYTHONPATH:+:\$PYTHONPATH}"
EOF
)
  if [[ -n "$REF_VENV_REL" ]]; then
    ENV_BLOCK+=$'\n'"[[ -f \"\$UNITREE_WS/$REF_VENV_REL/bin/activate\" ]] && source \"\$UNITREE_WS/$REF_VENV_REL/bin/activate\""
  fi
fi
# Rewrite any absolute reference to unitree's home so the block is
# portable to each new account (in case it was hardcoded instead of
# using $HOME).
ENV_BLOCK="${ENV_BLOCK//$REF_HOME/\$HOME}"

MARK_BEGIN="# >>> unitree_sdk2_python / cyclonedds setup (managed by reset_users.sh) >>>"
MARK_END="# <<< unitree_sdk2_python / cyclonedds setup (managed by reset_users.sh) <<<"

for u in "${USERS[@]}"; do
  echo "== $u =="

  # 1. Prune the account if it already exists.
  if id "$u" &>/dev/null; then
    echo "  exists -> pruning"
    pkill -KILL -u "$u" 2>/dev/null || true
    sleep 1
    userdel -r "$u" 2>/dev/null || userdel -f -r "$u"
  else
    echo "  does not exist -> nothing to prune"
  fi

  # 2. Recreate the account with no sudo rights.
  useradd -m -s "$SHELL_BIN" "$u"
  for g in sudo wheel admin; do
    gpasswd -d "$u" "$g" >/dev/null 2>&1 || true
  done
  printf '%s:%s\n' "$u" "$TEILNEHMER_PASSWORD" | chpasswd

  u_home="$(getent passwd "$u" | cut -d: -f6 || true)"
  dest_ws="$u_home/$REF_WS_REL"

  # 3. Clone the unitree_sdk2_python / cyclonedds workspace.
  mkdir -p "$(dirname "$dest_ws")"
  rsync -a --exclude='.git' "$REF_WS/" "$dest_ws/"
  chown -R "$u:$u" "$u_home"

  # venvs and native builds embed absolute paths (shebangs in
  # .venv/bin/*, the venv activate scripts, etc.). Rewrite every
  # occurrence of the reference home with this user's home so the
  # copied environment actually works instead of pointing back at
  # unitree's files.
  grep -rlI --null "$REF_HOME" "$dest_ws" 2>/dev/null \
    | xargs -0 -r sed -i "s#$REF_HOME#$u_home#g"

  # 3b. Safety net: make sure ipython/jupyter/rich/pandas/numpy/opencv/
  # piper-tts/pydantic/openai are actually present in this account's venv
  # (not just assumed from the clone), and register a Jupyter kernel for
  # it. Cheap no-op when the reference venv already has everything.
  venv_activate="$(find "$dest_ws" -maxdepth 4 -type f -name activate -path '*/bin/activate' 2>/dev/null | head -1)"
  if [[ -n "$venv_activate" ]]; then
    venv_py="$(dirname "$venv_activate")/python3"
    # -l gives the target user a real login environment (HOME, etc.) so
    # pip's cache/config land in their own home, not root's.
    runuser -l "$u" -c "'$venv_py' -m pip install --quiet ipython jupyterlab notebook ipykernel rich pandas numpy opencv-python piper-tts pydantic openai" \
      || echo "  warning: pip safety-net install failed for $u (check network)" >&2
    runuser -l "$u" -c "'$venv_py' -m ipykernel install --user --name unitree_sdk2 --display-name 'Unitree SDK2 (g1 academy)'" \
      || echo "  warning: could not register Jupyter kernel for $u" >&2
  else
    echo "  warning: no Python venv found under $dest_ws; skipped package/kernel check" >&2
  fi
  piper_bin="$(find "$dest_ws" -maxdepth 4 -type f -name piper -path '*/bin/piper' 2>/dev/null | head -1)"
  piper_bin="${piper_bin:-piper}"

  # 4. Configure the shell environment to match unitree's, plus Piper.
  bashrc="$u_home/.bashrc"
  [[ -f "$bashrc" ]] || cp /etc/skel/.bashrc "$bashrc" 2>/dev/null || touch "$bashrc"
  if ! grep -qF "$MARK_BEGIN" "$bashrc"; then
    {
      echo ""
      echo "$MARK_BEGIN"
      echo "$ENV_BLOCK"
      echo "export G1_PIPER_BIN=\"$piper_bin\""
      echo "export G1_PIPER_VOICE_DIR=\"$SHARED_VOICE_DIR\""
      echo "$MARK_END"
    } >> "$bashrc"
  fi
  chown "$u:$u" "$bashrc"

  echo "  recreated, workspace cloned to $dest_ws"
done

echo "Done. Reset ${#USERS[@]} accounts: ${USERS[*]}"
echo "The configured SSH/login password was applied to every participant account."
