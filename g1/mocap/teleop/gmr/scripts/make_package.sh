#!/usr/bin/env bash
# 打包 Teleop Studio 为自包含、可重定位安装包（Linux）。
# 须在已就绪 gmr conda 环境的 Linux 上运行（同一平台只能打该平台的包）。
#
# 产物结构：
#   dist/TeleopStudio-linux/
#     env/                  conda-pack 打包的 gmr 环境（首次启动自动 conda-unpack 修复路径）
#     gmr/                  GMR 仓库源码
#     run_teleop_studio.sh  相对自身定位 env/ 与 gmr/ 的启动器
#   dist/TeleopStudio-linux-<ver>.tar.gz  可分发压缩包
#
# 安装：用户解压到任意目录，运行 ./run_teleop_studio.sh 即可，路径全自动确定。
set -euo pipefail

GMR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${VERSION:-$(date +%Y%m%d)}"
STAGE="$GMR_ROOT/dist/TeleopStudio-linux"
PKG_NAME="TeleopStudio-linux-$VERSION"

# 让 conda / conda-pack 可用——自动探测，不写死路径
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
# shellcheck disable=SC1091
source "$CONDA_ROOT/bin/activate" base

echo "[1/7] 检查 conda-pack"
# conda-pack 不能装到 base（与 conda 自带的 setuptools 冲突，报 RemoveError），
# 装到 gmr 环境里。
if ! conda run -n gmr python -c "import conda_pack" >/dev/null 2>&1; then
  echo "  conda-pack 未安装，安装到 gmr 环境（不装 base 避免 setuptools 冲突）..."
  conda install -y -n gmr -c conda-forge conda-pack || {
    echo "  conda install 失败，尝试 pip install conda-pack ..."
    conda run -n gmr pip install conda-pack
  }
fi

echo "[2/7] 清理并创建暂存目录 $STAGE"
rm -rf "$STAGE"; mkdir -p "$STAGE"

echo "[3/7] 打包 gmr conda 环境"
# 用 gmr 环境里的 conda-pack（避免 base 的 setuptools 冲突）
conda run -n gmr conda-pack -n gmr -o "$STAGE/env.tar.gz" --force --ignore-missing-files --ignore-editable-packages
mkdir -p "$STAGE/env"
tar -xzf "$STAGE/env.tar.gz" -C "$STAGE/env"
rm -f "$STAGE/env.tar.gz"

echo "[4/7] 写入 .pack_source_root 并创建 activate.d 钩子"
echo "$GMR_ROOT" > "$STAGE/env/.pack_source_root"

# 创建 activate.d 钩子：每次激活时自动重写 editable 路径（finder MAPPING + direct_url.json）
ACTIVATE_DIR="$STAGE/env/etc/conda/activate.d"
mkdir -p "$ACTIVATE_DIR"
cat > "$ACTIVATE_DIR/zz_editable_rewrite.sh" << 'HOOK_EOF'
#!/usr/bin/env bash
# 打包态：环境激活时自动重写 editable 包的源码路径。
# conda-unpack 只修复 conda 前缀（shebang/激活脚本），
# editable finder MAPPING 和 direct_url.json 仍指向打包机路径。
# 本钩子在每次激活时检查并重写（幂等：已重写则 grep 无匹配，sed 不执行）。

_pack_root="$CONDA_PREFIX/.pack_source_root"
if [ -f "$_pack_root" ]; then
  _old="$(cat "$_pack_root" 2>/dev/null || true)"
  _new="$(cd "$CONDA_PREFIX/.." && pwd)/gmr"
  if [ -n "${_old:-}" ] && [ -d "${_new:-}" ]; then
    _site="$(find "$CONDA_PREFIX/lib" -maxdepth 4 -type d -name site-packages 2>/dev/null | head -1 || true)"
    if [ -n "${_site:-}" ]; then
      # 1) 重写 __editable___*_finder.py 的 MAPPING 路径（Python 导入用）
      for _f in "$_site"/__editable___*_finder.py; do
        [ -f "$_f" ] || continue
        grep -q "$_old" "$_f" 2>/dev/null && sed -i "s|${_old}|${_new}|g" "$_f" || true
      done
      # 2) 重写 direct_url.json（pip show 报告的 Editable project location）
      for _j in "$_site"/*.dist-info/direct_url.json; do
        [ -f "$_j" ] || continue
        grep -q "$_old" "$_j" 2>/dev/null && sed -i "s|${_old}|${_new}|g" "$_j" || true
      done
      # 3) 删除老式 egg-link（如有）
      find "$_site" -name '*.egg-link' -delete 2>/dev/null || true
    fi
  fi
fi
unset _pack_root _old _new _site _f _j
HOOK_EOF
chmod +x "$ACTIVATE_DIR/zz_editable_rewrite.sh"

echo "[5/7] 复制 GMR 仓库到 $STAGE/gmr"
rsync -a --delete \
  --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
  --exclude='dist' --exclude='build' --exclude='*.egg-info' \
  --exclude='.venv' --exclude='.idea' --exclude='node_modules' \
  --exclude='.workbuddy' --exclude='.vscode' --exclude='output' \
  --exclude='.pytest_cache' --exclude='assets/body_models' \
  --exclude='MUJOCO_LOG.TXT' --exclude='_run_output.txt' --exclude='tmpajb39ryj.xml' \
  "$GMR_ROOT"/ "$STAGE/gmr"/

echo "[6/7] 写入启动器"
cp "$GMR_ROOT/scripts/run_teleop_studio.sh" "$STAGE/run_teleop_studio.sh"
chmod +x "$STAGE/run_teleop_studio.sh"
cat > "$STAGE/PACKAGE_INFO.txt" <<EOF
Teleop Studio Linux
source_revision=$(git -C "$GMR_ROOT" rev-parse HEAD 2>/dev/null || printf unknown)
generated_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)
preview_defaults_ms=default:400,low_latency:100
history_window_frames=2
SMPL-X body models are not bundled; Moretion teleoperation does not need them.
EOF

echo "[7/7] 生成分发包 dist/$PKG_NAME.tar.gz"
# 打包内容（不含外层目录），解压后 env/ gmr/ run_teleop_studio.sh 直接在当前目录
tar -czf "$GMR_ROOT/dist/$PKG_NAME.tar.gz" -C "$STAGE" .
(
  cd "$GMR_ROOT/dist"
  sha256sum "$PKG_NAME.tar.gz" > "$PKG_NAME.tar.gz.sha256"
)
echo "完成：$GMR_ROOT/dist/$PKG_NAME.tar.gz"
echo "安装：解压后运行 ./run_teleop_studio.sh（首次自动 conda-unpack 修复路径）"
