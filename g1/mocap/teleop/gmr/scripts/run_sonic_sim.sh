#!/usr/bin/env bash
# SONIC MuJoCo 仿真启动器（Ubuntu/Linux）。
#
# 支持两种目录结构：
#   开发态：GMR_ROOT/scripts/run_sonic_sim.sh + conda 环境 gmr
#   打包态：<根目录>/{env,gmr}/，其中 env 是 conda-pack 解压后的环境
#
# 启动器会自动完成：conda-pack 路径修复、DDS 原生库路径设置、
# GMR 与 Unitree SDK Python 源码路径设置，然后把所有参数转交给
# scripts/run_sonic_sim.py。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

find_package_root() {
  local candidate="$1"
  while [ "$candidate" != "/" ]; do
    if [ -d "$candidate/env" ] && [ -d "$candidate/gmr" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
    candidate="$(dirname "$candidate")"
  done
  return 1
}

PKG_ROOT="$(find_package_root "$SCRIPT_DIR" || true)"
if [ -n "$PKG_ROOT" ]; then
  ENV_DIR="$PKG_ROOT/env"
  GMR_ROOT="$PKG_ROOT/gmr"
else
  ENV_DIR=""
  GMR_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
fi

if [ ! -f "$GMR_ROOT/scripts/run_sonic_sim.py" ]; then
  echo "[error] 未找到 GMR 仿真入口：$GMR_ROOT/scripts/run_sonic_sim.py" >&2
  exit 1
fi

if [ -n "$ENV_DIR" ]; then
  # conda-pack 环境中的 shebang 和动态路径在首次移动后需要修复。
  # conda-unpack 是幂等的；标记文件避免每次启动重复执行。
  if [ ! -x "$ENV_DIR/bin/python" ]; then
    echo "[error] 打包环境缺少可执行文件：$ENV_DIR/bin/python" >&2
    exit 1
  fi
  UNPACK_MARKER="$ENV_DIR/.conda-unpacked"
  if [ ! -f "$UNPACK_MARKER" ] || [ "$(cat "$UNPACK_MARKER" 2>/dev/null || true)" != "$ENV_DIR" ]; then
    set +u
    if [ -f "$ENV_DIR/bin/activate" ]; then
      # shellcheck disable=SC1091
      source "$ENV_DIR/bin/activate"
    fi
    if ! command -v conda-unpack >/dev/null 2>&1; then
      echo "[error] 找不到 conda-unpack，请确认 env 由 conda-pack 生成" >&2
      exit 1
    fi
    conda-unpack
    printf '%s\n' "$ENV_DIR" > "$UNPACK_MARKER"
    conda deactivate >/dev/null 2>&1 || true
    set -u
  fi
  PYTHON_BIN="$ENV_DIR/bin/python"
else
  # 开发态优先复用当前已激活的 gmr；否则自动探测常见 conda 安装位置。
  if [ "${CONDA_DEFAULT_ENV:-}" = "gmr" ] && [ -x "${CONDA_PREFIX:-}/bin/python" ]; then
    PYTHON_BIN="$CONDA_PREFIX/bin/python"
  else
    CONDA_ROOT="${CONDA_ROOT:-}"
    if [ -z "$CONDA_ROOT" ] && command -v conda >/dev/null 2>&1; then
      CONDA_ROOT="$(conda info --base 2>/dev/null || true)"
    fi
    if [ -z "$CONDA_ROOT" ] || [ ! -x "$CONDA_ROOT/bin/conda" ]; then
      user_home="${HOME:-}"
      for candidate in "$user_home/miniconda3" "$user_home/anaconda3" \
        "$user_home/.holosoma_deps/miniconda3" /opt/miniconda3 /opt/anaconda3 \
        /usr/local/miniconda3 /usr/local/anaconda3; do
        if [ -x "$candidate/bin/conda" ]; then
          CONDA_ROOT="$candidate"
          break
        fi
      done
    fi
    if [ -z "$CONDA_ROOT" ] || [ ! -x "$CONDA_ROOT/bin/conda" ]; then
      echo "[error] 未找到 conda，请设置 CONDA_ROOT，或先激活 gmr 环境" >&2
      exit 1
    fi
    set +u
    # shellcheck disable=SC1091
    source "$CONDA_ROOT/bin/activate" gmr
    set -u
    PYTHON_BIN="$CONDA_PREFIX/bin/python"
  fi
fi

DDS_DIR="$GMR_ROOT/sonic_sim/lib"
if [ ! -f "$DDS_DIR/libddsc.so" ]; then
  echo "[error] 未找到 SONIC DDS 库：$DDS_DIR/libddsc.so" >&2
  exit 1
fi

# 项目自带 libddsc.so 必须排在系统库之前；Unitree SDK 源码也随 GMR 仓库提供。
export CYCLONEDDS_HOME="$GMR_ROOT/sonic_sim"
export LD_LIBRARY_PATH="$DDS_DIR:${ENV_DIR:+$ENV_DIR/lib:}${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$GMR_ROOT:$GMR_ROOT/third_party/unitree_sdk2_python:${PYTHONPATH:-}"

if [ -n "$ENV_DIR" ]; then
  glibc_version="$(getconf GNU_LIBC_VERSION 2>/dev/null | awk '{print $2}' || true)"
  if [ -n "$glibc_version" ] && [ "$(printf '%s\n' "2.28" "$glibc_version" | sort -V | head -n 1)" != "2.28" ]; then
    echo "[error] 打包版至少需要 glibc 2.28（当前 $glibc_version）" >&2
    exit 1
  fi

  case " $* " in
    *" --no-onscreen "*) ;;
    *)
      if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
        echo "[warn] 未检测到桌面显示会话；请使用 --no-onscreen，或在 Ubuntu Desktop 会话中运行" >&2
      fi
      ;;
  esac
fi

if [ ! -x "$PYTHON_BIN" ]; then
  echo "[error] Python 不可执行：$PYTHON_BIN" >&2
  exit 1
fi

cd "$GMR_ROOT"
exec "$PYTHON_BIN" "$GMR_ROOT/scripts/run_sonic_sim.py" "$@"
