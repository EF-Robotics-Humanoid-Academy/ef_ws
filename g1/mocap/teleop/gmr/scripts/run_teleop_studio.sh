#!/usr/bin/env bash
# Teleop Studio 启动器 (Linux/Ubuntu)——开发态与打包态通用，路径相对自身自动确定。
#
# 工作模式（按同目录结构自动判定）：
#   打包态：本脚本旁存在 env/（conda-pack 打包环境）与 gmr/（仓库）。
#          首次运行自动执行 conda-unpack 修复 shebang/激活（幂等，只跑一次）。
#   开发态：无 env/ 同级目录，回退到 conda 激活 gmr 环境；仓库取上级目录。
#
# 不论哪种模式：
#   1. 激活/定位 Python 环境；
#   2. 把 PySide6 自带 Qt6 库目录前置到 LD_LIBRARY_PATH，防止系统 Qt6
#      （如 Ubuntu 22.04 的 6.4.2）被优先加载导致
#      "undefined symbol _ZN14QObjectPrivateC2Ei, Qt_6_PRIVATE_API"。
#      （该变量必须在导入 PySide6 前的 loader 阶段设置。）
#   3. 启动主应用。
#
# 可用环境变量覆盖：CONDA_ROOT（开发态 conda 安装目录，自动探测）
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- 1) 定位打包根目录（向上查找 env/ + gmr/ 目录对）---
# 从顶层 run_teleop_studio.sh 运行时 SCRIPT_DIR 即根目录；
# 从 gmr/scripts/ 内的仓库副本运行时需向上查找。
PKG_ROOT="$SCRIPT_DIR"
while [ "$PKG_ROOT" != "/" ]; do
  if [ -d "$PKG_ROOT/env" ] && [ -d "$PKG_ROOT/gmr" ]; then
    break
  fi
  PKG_ROOT="$(dirname "$PKG_ROOT")"
done

# --- 2) 定位 Python 环境 ---
if [ -d "$PKG_ROOT/env" ] && [ -d "$PKG_ROOT/gmr" ]; then
  # 打包态：用随附的 conda-pack 环境
  ENV_DIR="$PKG_ROOT/env"
  # conda activate 脚本会引用 CONDA_PREFIX 等可能未设置的变量，
  # 临时关闭 -u 避免 set -u 触发退出
  set +u
  if [ ! -f "$ENV_DIR/.conda-unpacked" ]; then
    # shellcheck disable=SC1091
    source "$ENV_DIR/bin/activate"
    conda-unpack && touch "$ENV_DIR/.conda-unpacked"
    conda deactivate 2>/dev/null || true
    # editable 路径重写由 activate.d/zz_editable_rewrite.sh 钩子自动处理，
    # 每次激活时检查并重写 finder MAPPING + direct_url.json（幂等）。
  fi
  # shellcheck disable=SC1091
  source "$ENV_DIR/bin/activate"
  set -u
else
  # 开发态：自动探测 conda 安装目录
  if [ -z "${CONDA_ROOT:-}" ]; then
    if command -v conda >/dev/null 2>&1; then
      CONDA_ROOT="$(conda info --base 2>/dev/null)"
    fi
    if [ -z "${CONDA_ROOT:-}" ] || [ ! -d "${CONDA_ROOT:-}" ]; then
      for _d in "$HOME/miniconda3" "$HOME/anaconda3" \
                "$HOME/.holosoma_deps/miniconda3" \
                /opt/miniconda3 /opt/anaconda3 \
                /usr/local/miniconda3 /usr/local/anaconda3; do
        if [ -x "$_d/bin/conda" ]; then CONDA_ROOT="$_d"; break; fi
      done
    fi
  fi
  if [ -z "${CONDA_ROOT:-}" ] || [ ! -x "${CONDA_ROOT}/bin/conda" ]; then
    echo "[error] 未找到 conda，请设置 CONDA_ROOT 环境变量" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1091
  source "$CONDA_ROOT/bin/activate" gmr
  set -u
fi

# --- 3) 定位 GMR 仓库 ---
if [ -d "$PKG_ROOT/gmr" ]; then
  GMR_ROOT="$PKG_ROOT/gmr"
else
  GMR_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
fi
cd "$GMR_ROOT"

# 让随附的源码包可被导入（PYTHONPATH 作为 editable finder 之外的额外路径）
export PYTHONPATH="$GMR_ROOT:${PYTHONPATH:-}"

# --- 3) PySide6 自带 Qt6 库目录前置到 LD_LIBRARY_PATH ---
QTLIB="$(python -c 'import PySide6,os;print(os.path.join(os.path.dirname(PySide6.__file__),"Qt","lib"))' 2>/dev/null || true)"
if [ -n "${QTLIB:-}" ]; then
  export LD_LIBRARY_PATH="${QTLIB}:${LD_LIBRARY_PATH:-}"
else
  echo "[warn] 未定位到 PySide6 的 Qt/lib 目录，跳过 LD_LIBRARY_PATH 修复" >&2
fi

exec python -m teleop_studio.app "$@"
