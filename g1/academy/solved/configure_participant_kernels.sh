#!/usr/bin/env bash
set -euo pipefail

# Make the normal Jupyter "Python 3" kernel the tested Unitree SDK/DDS
# runtime for every academy account. This overrides JupyterHub's bare hub
# environment, which does not carry NumPy or the Unitree SDK.
# Usage: sudo bash configure_participant_kernels.sh

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
REF_PY="${REF_PY:-/home/unitree/.guv/envs/unitree}"
CYCLONEDDS_HOME="${CYCLONEDDS_HOME:-/home/unitree/unitree_ros2/cyclonedds_ws/install/cyclonedds}"
OPENAIAPI_DIR="${OPENAIAPI_DIR:-/home/unitree/EF/ef_ws/g1/modules/scripts/openaiapi}"
PIPER_BIN="${PIPER_BIN:-$REF_PY/bin/piper}"
PIPER_VOICE_DIR="${PIPER_VOICE_DIR:-/opt/academy-tools/piper-voices}"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo bash $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
[[ -x "$REF_PY/bin/python" && -f "$CYCLONEDDS_HOME/lib/libddsc.so" ]] || {
  echo "The Unitree Python runtime or CycloneDDS installation is missing." >&2; exit 1;
}
[[ -f "$PIPER_VOICE_DIR/en_US-lessac-medium/en_US-lessac-medium.onnx" ]] || {
  echo "The shared Piper en_US-lessac-medium voice is missing from $PIPER_VOICE_DIR." >&2; exit 1;
}

for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing account: $user" >&2; exit 1; }
  home_dir="$(getent passwd "$user" | cut -d: -f6)"
  sdk_dir="$home_dir/unitree_sdk2_python"
  [[ -d "$sdk_dir/unitree_sdk2py" ]] || { echo "Missing SDK checkout for $user" >&2; exit 1; }

  # Unitree SDK2's interface-specific ChannelConfig hard-codes /tmp/cdds.LOG.
  # That becomes unwritable as soon as the first user creates it. Give every
  # account a private trace path before any ChannelFactory is initialized.
  dds_log_dir="$home_dir/.cache/cyclonedds"
  install -d -m 0700 "$dds_log_dir"
  channel_config="$sdk_dir/unitree_sdk2py/core/channel_config.py"
  [[ -f "$channel_config" ]] || { echo "Missing SDK channel config for $user" >&2; exit 1; }
  sed -i "s|/tmp/cdds\.LOG|$dds_log_dir/cdds.LOG|g" "$channel_config"
  chown "$user:$user" "$dds_log_dir"

  # sdk_wrapper.py follows Piper's conventional per-user voice location.
  # Keep one root-owned, read-only shared copy and make it visible at that
  # location without duplicating the large model for every participant.
  piper_home="$home_dir/.local/share/piper"
  voice_path="$piper_home/voices"
  install -d -m 0755 "$piper_home"
  if [[ -L "$voice_path" ]]; then
    ln -sfn "$PIPER_VOICE_DIR" "$voice_path"
    chown -h "$user:$user" "$voice_path"
  elif [[ -d "$voice_path" ]]; then
    # Preserve a participant's existing local voices, adding the academy
    # voice if necessary instead of replacing their directory.
    cp -a "$PIPER_VOICE_DIR/." "$voice_path/"
    chown -R "$user:$user" "$voice_path"
  elif [[ -e "$voice_path" ]]; then
    echo "Cannot configure Piper for $user: $voice_path is not a directory or symlink." >&2
    exit 1
  else
    ln -s "$PIPER_VOICE_DIR" "$voice_path"
    chown -h "$user:$user" "$voice_path"
  fi
  chown "$user:$user" "$piper_home"

  bin_dir="$home_dir/.local/bin"
  install -d -m 0755 "$bin_dir"
  wrapper="$bin_dir/academy-unitree-kernel"
  cat >"$wrapper" <<EOF
#!/usr/bin/env bash
[ -f "\$HOME/.g1-unitree-env" ] && source "\$HOME/.g1-unitree-env"
[ -f "\$HOME/.academy-api.env" ] && source "\$HOME/.academy-api.env"
[ -f "\$HOME/.academy-tools-env" ] && source "\$HOME/.academy-tools-env"
exec "$REF_PY/bin/python" -m ipykernel_launcher "\$@"
EOF
  chmod 0700 "$wrapper"

  for kernel_name in python3 unitree_sdk2; do
    kernel_dir="$home_dir/.local/share/jupyter/kernels/$kernel_name"
    install -d -m 0755 "$kernel_dir"
    display_name='Python 3 (Unitree SDK2)'
    [[ "$kernel_name" == unitree_sdk2 ]] && display_name='Unitree SDK2 (g1 academy)'
    cat >"$kernel_dir/kernel.json" <<EOF
{
  "argv": ["$wrapper", "-f", "{connection_file}"],
  "display_name": "$display_name",
  "language": "python",
  "env": {
    "UNITREE_SDK2_HOME": "$sdk_dir",
    "CYCLONEDDS_HOME": "$CYCLONEDDS_HOME",
    "CycloneDDS_ROOT": "$CYCLONEDDS_HOME",
    "LD_LIBRARY_PATH": "$CYCLONEDDS_HOME/lib",
    "PYTHONPATH": "$sdk_dir:$OPENAIAPI_DIR",
    "G1_OPENAIAPI_DIR": "$OPENAIAPI_DIR",
    "G1_PIPER_BIN": "$PIPER_BIN",
    "G1_PIPER_VOICE_DIR": "$PIPER_VOICE_DIR",
    "OLLAMA_HOST": "http://127.0.0.1:11434"
  }
}
EOF
  done
  # Do not recursively chown ~/.local: it contains the shared Piper-voice
  # symlink above.  Limit ownership changes to files created by this script.
  chown "$user:$user" "$bin_dir" "$wrapper"
  chown -R "$user:$user" "$home_dir/.local/share/jupyter"

  runuser -u "$user" -- env HOME="$home_dir" \
    LD_LIBRARY_PATH="$CYCLONEDDS_HOME/lib" PYTHONPATH="$sdk_dir:$OPENAIAPI_DIR" \
    "$REF_PY/bin/python" -c 'import numpy, cyclonedds, unitree_sdk2py; from unitree_sdk2py.core.channel import ChannelFactoryInitialize; ChannelFactoryInitialize(0, "eth0")'
done

hub_config="/etc/jupyterhub/jupyterhub_config.py"
if [[ -f "$hub_config" ]]; then
  hub_mark_begin='# >>> academy participant kernel priority (managed) >>>'
  hub_mark_end='# <<< academy participant kernel priority (managed) <<<'
  sed -i "\\|$hub_mark_begin|,\\|$hub_mark_end|d" "$hub_config"
  {
    printf '\n%s\n' "$hub_mark_begin"
    printf '%s\n' "# Prefer each user's Python 3 Unitree SDK2 kernel over the Hub venv kernel."
    printf '%s\n' "c.Spawner.environment = {'JUPYTER_PREFER_ENV_PATH': '0'}"
    printf '%s\n' "# Permit JupyterLab's sandboxed local-HTML viewer to load academy slide assets."
    printf '%s\n' "c.Spawner.args = ['--ServerApp.default_kernel_name=python3', '--ServerApp.allow_origin=*']"
    printf '%s\n' "$hub_mark_end"
  } >>"$hub_config"
  systemctl restart jupyterhub.service
fi

echo "Configured and verified Unitree-backed default kernels for $NUM_USERS participants."
