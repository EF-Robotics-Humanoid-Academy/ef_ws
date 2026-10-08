#!/usr/bin/env bash
set -euo pipefail

# PAM JupyterHub for Ubuntu 20.04 (where current TLJH cannot be installed).
# Serves at http://<host>:8000 and spawns notebooks as the existing Linux
# participant accounts; it never grants them sudo privileges.

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
ADMIN_USER="${ADMIN_USER:-unitree}"
HUB_ROOT="${HUB_ROOT:-/opt/jupyterhub}"
HUB_PORT="${HUB_PORT:-8000}"
CHP_VERSION="${CHP_VERSION:-4.5.3}"
NODE_VERSION="${NODE_VERSION:-20.19.0}"
NODE_ARCHIVE="node-v${NODE_VERSION}-linux-arm64.tar.xz"
NODE_SHA256="${NODE_SHA256:-dbe339e55eb393955a213e6b872066880bb9feceaa494f4d44c7aac205ec2ab9}"

[[ "$(id -u)" -eq 0 ]] || { echo "Run as root: sudo $0" >&2; exit 1; }
[[ "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] && (( NUM_USERS <= 100 )) || { echo "NUM_USERS must be 1..100" >&2; exit 2; }
[[ "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] && (( ${#USER_PREFIX} <= 24 )) || { echo "USER_PREFIX must be a safe, non-empty Linux username prefix (max 24 characters)" >&2; exit 2; }
id "$ADMIN_USER" &>/dev/null || { echo "Missing admin account: $ADMIN_USER" >&2; exit 1; }
command -v apt-get >/dev/null || { echo "This installer requires apt-get." >&2; exit 1; }

users=()
for ((i=1; i<=NUM_USERS; i++)); do
  user="${USER_PREFIX}${i}"
  id "$user" &>/dev/null || { echo "Missing participant account: $user (run provision_participants.sh first)" >&2; exit 1; }
  users+=("$user")
done

export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y python3-venv curl ca-certificates xz-utils

install -d -m 0755 "$HUB_ROOT"
if [[ ! -x "$HUB_ROOT/venv/bin/jupyterhub" ]]; then
  python3 -m venv "$HUB_ROOT/venv"
  "$HUB_ROOT/venv/bin/python" -m pip install --upgrade pip wheel
  # This release supports Ubuntu 20.04's Python 3.8.
  "$HUB_ROOT/venv/bin/python" -m pip install 'jupyterhub==3.1.1' 'jupyterlab==3.6.8'
fi

# Ubuntu 20.04 ships Node 10, which cannot run the current proxy dependency
# tree. Keep a verified, private Node LTS runtime under the Hub directory;
# do not alter the system Node installation used by other robot software.
node_dir="$HUB_ROOT/node"
if [[ ! -x "$node_dir/bin/node" ]]; then
  node_tmp="$(mktemp --tmpdir jupyterhub-node.XXXXXX.tar.xz)"
  trap 'rm -f "$node_tmp"' EXIT
  curl -fsSL "https://nodejs.org/dist/v${NODE_VERSION}/${NODE_ARCHIVE}" -o "$node_tmp"
  printf '%s  %s\n' "$NODE_SHA256" "$node_tmp" | sha256sum --check --status
  install -d -m 0755 "$node_dir"
  tar -xJf "$node_tmp" --strip-components=1 -C "$node_dir"
  rm -f "$node_tmp"
  trap - EXIT
fi
if [[ "$("$node_dir/bin/node" --version)" != "v$NODE_VERSION" ]]; then
  echo "Unexpected Node runtime at $node_dir; expected v$NODE_VERSION." >&2
  exit 1
fi
# npm's launcher uses /usr/bin/env node, so put the private Node binary first
# or it would accidentally execute with Ubuntu 20.04's system Node 10.
PATH="$node_dir/bin:$PATH" "$node_dir/bin/npm" --prefix "$node_dir" install --global "configurable-http-proxy@$CHP_VERSION"
proxy_bin="$node_dir/bin/configurable-http-proxy"

install -d -m 0755 /etc/jupyterhub
allowed_users="$(printf "'%s', " "${users[@]}")"
allowed_users="${allowed_users%, }"
cat >/etc/jupyterhub/jupyterhub_config.py <<EOF
# Managed by setup_jupyterhub_focal.sh
c.JupyterHub.bind_url = 'http://:${HUB_PORT}'
c.JupyterHub.hub_bind_url = 'http://127.0.0.1:8081'
c.JupyterHub.authenticator_class = 'jupyterhub.auth.PAMAuthenticator'
c.Authenticator.allowed_users = {${allowed_users}}
c.Authenticator.admin_users = {'${ADMIN_USER}'}
c.LocalAuthenticator.create_system_users = False
c.Spawner.cmd = ['${HUB_ROOT}/venv/bin/jupyterhub-singleuser']
# The Hub itself runs in a minimal venv. Prefer each participant's kernel
# specs (not the Hub venv's built-in python3), so existing notebooks whose
# metadata says "python3" use the Unitree SDK2 runtime installed per user.
c.Spawner.environment = {'JUPYTER_PREFER_ENV_PATH': '0'}
# JupyterLab's HTML viewer renders local HTML in a sandboxed frame with an
# opaque origin. Allow that viewer to load the academy deck's embedded/media
# assets; authentication and the Hub user allowlist still protect the server.
c.Spawner.args = ['--ServerApp.default_kernel_name=python3', '--ServerApp.allow_origin=*']
c.Spawner.default_url = '/lab'
c.JupyterHub.proxy_cmd = ['${proxy_bin}']
c.JupyterHub.shutdown_on_logout = False
EOF

cat >/etc/systemd/system/jupyterhub.service <<EOF
[Unit]
Description=JupyterHub (PAM academy users)
After=network.target

[Service]
Type=simple
User=root
WorkingDirectory=/etc/jupyterhub
Environment="PATH=${HUB_ROOT}/venv/bin:${node_dir}/bin:/usr/local/bin:/usr/bin:/bin"
ExecStart=${HUB_ROOT}/venv/bin/jupyterhub -f /etc/jupyterhub/jupyterhub_config.py
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable --now jupyterhub.service
systemctl --no-pager --full status jupyterhub.service
echo "JupyterHub is ready at http://<this-host>:${HUB_PORT}/"
