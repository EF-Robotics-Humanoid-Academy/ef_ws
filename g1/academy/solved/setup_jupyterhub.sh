#!/usr/bin/env bash
set -euo pipefail

# setup_jupyterhub.sh — install and configure JupyterHub (via "The Littlest
# JupyterHub", TLJH) on the headless Jetson so every teilnehmerN account
# gets its own browser-based JupyterLab, with the kernel actually running
# on the Jetson under that student's own Linux account and cloned venv.
#
# Architecture:
#   - Auth:    PAMAuthenticator against the existing teilnehmerN accounts
#              (the ones reset_users.sh creates), using their configured
#              Linux passwords.
#   - Serving: one shared JupyterLab install (TLJH's /opt/tljh/user env)
#              handles the web UI for everyone — that part does NOT
#              multiply per student.
#   - Kernels: SystemdSpawner starts each student's single-user server (and
#              their notebook kernels) as a transient systemd scope running
#              as *their* Linux user, using the unitree_sdk2 kernel already
#              registered against their own cloned venv (see
#              reset_users.sh) — so only the kernel processes multiply,
#              each cgroup-limited and idle-culled below.
#   - Robot access: intentionally NOT arbitrated here. Multiple students'
#     kernels can send actuation commands to the same physical robot at
#     the same time; coordinate whose turn it is verbally in class.
#
# Usage: sudo ./setup_jupyterhub.sh
#
# Re-running is safe: the TLJH bootstrap upgrades an existing install, and
# every tljh-config call below is idempotent.

usage() {
  cat <<'EOF'
Usage: sudo setup_jupyterhub.sh [--help]

Install or reconfigure TLJH for the academy accounts. Configuration can be
overridden with NUM_USERS, USER_PREFIX, ADMIN_USER, MEM_LIMIT, CPU_LIMIT,
IDLE_TIMEOUT, and CULL_EVERY.
EOF
}

case "${1:-}" in
  "") ;;
  -h|--help) usage; exit 0 ;;
  *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
esac

if [[ "$(id -u)" -ne 0 ]]; then
  echo "This script must be run as root (e.g. sudo $0)." >&2
  exit 1
fi

# Current TLJH deliberately requires Ubuntu 22.04+.  Keep the same entry
# point usable on the Jetson's Ubuntu 20.04 by delegating to the compatible
# PAM JupyterHub service beside this script.
if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  if [[ "${VERSION_ID:-}" == "20.04" ]]; then
    exec /bin/bash "$(cd "$(dirname "$0")" && pwd)/setup_jupyterhub_focal.sh"
  fi
fi

NUM_USERS="${NUM_USERS:-13}"
USER_PREFIX="${USER_PREFIX:-teilnehmer}"
ADMIN_USER="${ADMIN_USER:-unitree}"
MEM_LIMIT="${MEM_LIMIT:-1.5G}"      # per-student server memory cap
CPU_LIMIT="${CPU_LIMIT:-1}"         # per-student server CPU core cap
IDLE_TIMEOUT="${IDLE_TIMEOUT:-1800}" # seconds of inactivity before culling (30 min)
CULL_EVERY="${CULL_EVERY:-300}"     # how often the culler checks (5 min)
TLJH_BOOTSTRAP_URL="${TLJH_BOOTSTRAP_URL:-https://raw.githubusercontent.com/jupyterhub/the-littlest-jupyterhub/main/bootstrap/bootstrap.py}"
TLJH_BOOTSTRAP_SHA256="${TLJH_BOOTSTRAP_SHA256:-}"

if [[ ! "$NUM_USERS" =~ ^[1-9][0-9]*$ ]] || ((NUM_USERS > 100)); then
  echo "NUM_USERS must be an integer between 1 and 100." >&2
  exit 1
fi
if [[ ! "$USER_PREFIX" =~ ^[a-z_][a-z0-9_-]*$ ]] || ((${#USER_PREFIX} > 24)); then
  echo "USER_PREFIX must be a safe lowercase Linux account prefix (maximum 24 characters)." >&2
  exit 1
fi
if [[ ! "$ADMIN_USER" =~ ^[a-z_][a-z0-9_-]*[$]?$ ]]; then
  echo "ADMIN_USER is not a valid Linux account name." >&2
  exit 1
fi
if ! id "$ADMIN_USER" &>/dev/null; then
  echo "ADMIN_USER does not exist: $ADMIN_USER" >&2
  exit 1
fi
if [[ ! "$MEM_LIMIT" =~ ^[1-9][0-9]*([.][0-9]+)?[KMGTPE]?$ ]]; then
  echo "MEM_LIMIT must be a positive systemd size such as 1536M or 1.5G." >&2
  exit 1
fi
if [[ ! "$CPU_LIMIT" =~ ^[0-9]+([.][0-9]+)?$ || "$CPU_LIMIT" =~ ^0+([.]0+)?$ ]]; then
  echo "CPU_LIMIT must be a positive number of CPU cores." >&2
  exit 1
fi
for timing_name in IDLE_TIMEOUT CULL_EVERY; do
  timing_value="${!timing_name}"
  if [[ ! "$timing_value" =~ ^[1-9][0-9]*$ ]]; then
    echo "$timing_name must be a positive integer number of seconds." >&2
    exit 1
  fi
done
if [[ "$TLJH_BOOTSTRAP_URL" != https://* ]]; then
  echo "TLJH_BOOTSTRAP_URL must use HTTPS." >&2
  exit 1
fi

USERS=()
for ((i = 1; i <= NUM_USERS; i++)); do
  USERS+=("${USER_PREFIX}${i}")
done

# --- 1. Install (or upgrade) TLJH ---------------------------------------
if ! command -v tljh-config >/dev/null 2>&1; then
  echo "Installing The Littlest JupyterHub..."
  for command_name in curl python3; do
    command -v "$command_name" >/dev/null 2>&1 || {
      echo "Required command not found: $command_name" >&2
      exit 1
    }
  done
  bootstrap_file="$(mktemp --tmpdir tljh-bootstrap.XXXXXX.py)"
  trap 'rm -f "$bootstrap_file"' EXIT
  curl -fsSL --retry 3 --retry-delay 2 "$TLJH_BOOTSTRAP_URL" -o "$bootstrap_file"
  if [[ -n "$TLJH_BOOTSTRAP_SHA256" ]]; then
    command -v sha256sum >/dev/null 2>&1 || {
      echo "sha256sum is required when TLJH_BOOTSTRAP_SHA256 is set." >&2
      exit 1
    }
    printf '%s  %s\n' "$TLJH_BOOTSTRAP_SHA256" "$bootstrap_file" | sha256sum --check --status
  else
    echo "warning: TLJH bootstrap checksum is not pinned; set TLJH_BOOTSTRAP_SHA256 for verification" >&2
  fi
  python3 "$bootstrap_file" --admin "$ADMIN_USER"
  rm -f "$bootstrap_file"
  trap - EXIT
else
  echo "TLJH already installed; reconfiguring."
fi

# --- 2. Authentication: PAM against existing Linux accounts -------------
# Select PAM explicitly rather than relying on a TLJH version default. It
# checks the existing Linux accounts directly, so participant accounts created
# by reset_users.sh/provision_participants.sh work with no separate Hub user
# database.
# Only accounts explicitly allow-listed here may log in.
tljh-config set auth.type jupyterhub.auth.PAMAuthenticator
tljh-config add-item users.admin "$ADMIN_USER"
for u in "${USERS[@]}"; do
  if id "$u" &>/dev/null; then
    tljh-config add-item users.allowed "$u"
  else
    echo "warning: $u does not exist yet (run reset_users.sh first); not allow-listing." >&2
  fi
done

# --- 3. Per-student resource limits (SystemdSpawner + cgroups) ----------
tljh-config set limits.memory "$MEM_LIMIT"
tljh-config set limits.cpu "$CPU_LIMIT"

# --- 4. Idle culling: stop servers/kernels nobody is using --------------
tljh-config set services.cull.enabled true
tljh-config set services.cull.timeout "$IDLE_TIMEOUT"
tljh-config set services.cull.every "$CULL_EVERY"
tljh-config set services.cull.users true   # also stop the idle single-user server, not just its kernels

tljh-config reload

cat <<EOF

Done. JupyterHub is available at: http://<jetson-ip>/
(Configure HTTPS separately before exposing it beyond a trusted classroom network.)
Admin: $ADMIN_USER
Allowed students: ${USERS[*]}
Per-student limits: ${MEM_LIMIT} RAM, ${CPU_LIMIT} CPU core(s)
Idle culling: after ${IDLE_TIMEOUT}s inactivity, checked every ${CULL_EVERY}s

Each student's own "unitree_sdk2" kernel (registered by
provision_participants.sh against the tested shared Unitree runtime) will
show up automatically in their kernel picker — no extra kernel configuration
is needed here.

No robot-access arbitration is configured: multiple students' kernels can
send commands to the physical robot concurrently. Coordinate turns
verbally, or revisit this if it becomes a problem in practice.
EOF
