#!/usr/bin/env bash
# Build a relocatable, minimal SONIC MuJoCo simulator package for Ubuntu x86_64.
#
# The package intentionally does not install GMR itself. Installing setup.py would
# pull in Teleop Studio, SMPL-X, PyTorch, CUDA, and other unrelated dependencies.
# CycloneDDS 0.10.2 is copied from the known-working gmr environment because its
# Python extension must match the bundled sonic_sim/lib/libddsc.so ABI.
#
# Output:
#   dist/SonicSim-linux-x86_64/
#   dist/SonicSim-linux-x86_64-<version>.tar.gz
#
# Configuration through environment variables:
#   VERSION=20260824          Archive version (defaults to current date)
#   SOURCE_ENV=gmr           Conda environment providing CycloneDDS 0.10.2
#   CONDA_ROOT=/path/to/conda Override conda auto-detection
#   OFFLINE_BUILD=1          Reuse runtime packages from SOURCE_ENV without pip downloads
#   KEEP_BUILD_ENV=1         Keep the clean build environment after packaging
set -Eeuo pipefail

GMR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VERSION="${VERSION:-$(date +%Y%m%d)}"
SOURCE_ENV="${SOURCE_ENV:-gmr}"
KEEP_BUILD_ENV="${KEEP_BUILD_ENV:-0}"
OFFLINE_BUILD="${OFFLINE_BUILD:-0}"
BUILD_ROOT="$GMR_ROOT/build/sonic-package"
BUILD_ENV="$BUILD_ROOT/env"
STAGE="$GMR_ROOT/dist/SonicSim-linux-x86_64"
PKG_NAME="SonicSim-linux-x86_64-$VERSION"
PYTHON_VERSION="3.11"
TARGET_GLIBC="2.28"

NUMPY_VERSION="2.4.6"
SCIPY_VERSION="1.17.1"
MUJOCO_VERSION="3.9.0"
PYYAML_VERSION="6.0.3"
CYCLONEDDS_VERSION="0.10.2"
export PYTHONNOUSERSITE=1

usage() {
  sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
}

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
  usage
  exit 0
fi
if [ "$#" -ne 0 ]; then
  echo "[error] 不支持的参数：$*" >&2
  usage >&2
  exit 2
fi

if [ "$(uname -s)" != "Linux" ] || [ "$(uname -m)" != "x86_64" ]; then
  echo "[error] 该脚本仅支持 Ubuntu Linux x86_64 构建机" >&2
  exit 1
fi
OS_PRETTY_NAME="Linux"
if [ -r /etc/os-release ]; then
  OS_ID="$(. /etc/os-release; printf '%s' "${ID:-}")"
  OS_PRETTY_NAME="$(. /etc/os-release; printf '%s' "${PRETTY_NAME:-Linux}")"
  if [ "$OS_ID" != "ubuntu" ]; then
    echo "[error] 该脚本须在 Ubuntu 构建机上运行（当前：$OS_PRETTY_NAME）" >&2
    exit 1
  fi
fi

for command_name in file rsync tar gzip sha256sum objdump; do
  if ! command -v "$command_name" >/dev/null 2>&1; then
    echo "[error] 缺少系统命令：$command_name" >&2
    exit 1
  fi
done

# Auto-detect conda without assuming a machine-specific install path.
if [ -z "${CONDA_ROOT:-}" ]; then
  if command -v conda >/dev/null 2>&1; then
    CONDA_ROOT="$(conda info --base 2>/dev/null || true)"
  fi
  if [ -z "${CONDA_ROOT:-}" ] || [ ! -x "$CONDA_ROOT/bin/conda" ]; then
    for candidate in "$HOME/miniconda3" "$HOME/anaconda3" \
      "$HOME/.holosoma_deps/miniconda3" /opt/miniconda3 /opt/anaconda3 \
      /usr/local/miniconda3 /usr/local/anaconda3; do
      if [ -x "$candidate/bin/conda" ]; then
        CONDA_ROOT="$candidate"
        break
      fi
    done
  fi
fi
if [ -z "${CONDA_ROOT:-}" ] || [ ! -x "$CONDA_ROOT/bin/conda" ]; then
  echo "[error] 未找到 conda，请设置 CONDA_ROOT" >&2
  exit 1
fi
CONDA_BIN="$CONDA_ROOT/bin/conda"

echo "[1/9] 检查源环境与 SONIC DDS ABI"
SOURCE_PYTHON="$("$CONDA_BIN" run -n "$SOURCE_ENV" python -c 'import sys; print(sys.executable)' | tail -n 1)"
if [ ! -x "$SOURCE_PYTHON" ]; then
  echo "[error] 无法定位 conda 环境 '$SOURCE_ENV' 的 Python" >&2
  exit 1
fi

SOURCE_INFO="$("$SOURCE_PYTHON" - <<'PY'
import importlib.metadata
import pathlib
import site
import sys

version = importlib.metadata.version("cyclonedds")
site_dir = pathlib.Path(site.getsitepackages()[0])
print(f"{sys.version_info.major}.{sys.version_info.minor}|{version}|{site_dir}")
PY
)"
IFS='|' read -r SOURCE_PYTHON_VERSION SOURCE_CYCLONEDDS_VERSION SOURCE_SITE <<< "$SOURCE_INFO"
if [ "$SOURCE_PYTHON_VERSION" != "$PYTHON_VERSION" ]; then
  echo "[error] '$SOURCE_ENV' 必须使用 Python $PYTHON_VERSION（当前 $SOURCE_PYTHON_VERSION）" >&2
  exit 1
fi
if [ "$SOURCE_CYCLONEDDS_VERSION" != "$CYCLONEDDS_VERSION" ]; then
  echo "[error] '$SOURCE_ENV' 必须安装 cyclonedds==$CYCLONEDDS_VERSION（当前 $SOURCE_CYCLONEDDS_VERSION）" >&2
  exit 1
fi
if [ ! -f "$GMR_ROOT/sonic_sim/lib/libddsc.so" ]; then
  echo "[error] 缺少 $GMR_ROOT/sonic_sim/lib/libddsc.so" >&2
  exit 1
fi
if [ ! -f "$GMR_ROOT/SONIC_SIM_USAGE.md" ]; then
  echo "[error] 缺少英文使用说明：$GMR_ROOT/SONIC_SIM_USAGE.md" >&2
  exit 1
fi
if [ ! -f "$GMR_ROOT/SONIC_SIM_USAGE.zh-CN.md" ]; then
  echo "[error] 缺少中文使用说明：$GMR_ROOT/SONIC_SIM_USAGE.zh-CN.md" >&2
  exit 1
fi
if ! file "$GMR_ROOT/sonic_sim/lib/libddsc.so" | grep -q 'x86-64'; then
  echo "[error] libddsc.so 不是 x86_64 动态库" >&2
  exit 1
fi

MAX_DDS_GLIBC="$(objdump -T "$GMR_ROOT/sonic_sim/lib/libddsc.so" \
  | grep -oE 'GLIBC_[0-9.]+' | sort -V | tail -n 1 || true)"
if [ -n "$MAX_DDS_GLIBC" ] \
  && [ "$(printf '%s\n' "GLIBC_$TARGET_GLIBC" "$MAX_DDS_GLIBC" | sort -V | tail -n 1)" != "GLIBC_$TARGET_GLIBC" ]; then
  echo "[error] libddsc.so 需要 $MAX_DDS_GLIBC，无法支持 Ubuntu 20.04" >&2
  exit 1
fi
CYCLONEDDS_CLAYER=("$SOURCE_SITE"/cyclonedds/_clayer*.so)
if [ "${#CYCLONEDDS_CLAYER[@]}" -ne 1 ] || [ ! -f "${CYCLONEDDS_CLAYER[0]}" ]; then
  echo "[error] '$SOURCE_ENV' 中缺少唯一的 CycloneDDS _clayer 扩展" >&2
  exit 1
fi
if ! file "${CYCLONEDDS_CLAYER[0]}" | grep -q 'x86-64'; then
  echo "[error] CycloneDDS _clayer 不是 x86_64 扩展" >&2
  exit 1
fi
MAX_CLAYER_GLIBC="$(objdump -T "${CYCLONEDDS_CLAYER[0]}" \
  | grep -oE 'GLIBC_[0-9.]+' | sort -V | tail -n 1 || true)"
if [ -n "$MAX_CLAYER_GLIBC" ] \
  && [ "$(printf '%s\n' "GLIBC_$TARGET_GLIBC" "$MAX_CLAYER_GLIBC" | sort -V | tail -n 1)" != "GLIBC_$TARGET_GLIBC" ]; then
  echo "[error] CycloneDDS _clayer 需要 $MAX_CLAYER_GLIBC，无法支持 Ubuntu 20.04" >&2
  exit 1
fi
if [ -n "$MAX_DDS_GLIBC" ]; then
  echo "  CycloneDDS: $SOURCE_CYCLONEDDS_VERSION, libddsc: $MAX_DDS_GLIBC, _clayer: ${MAX_CLAYER_GLIBC:-unknown}"
fi

echo "[2/9] 创建最小 Python $PYTHON_VERSION 环境"
rm -rf "$BUILD_ROOT" "$STAGE"
mkdir -p "$BUILD_ROOT/conda-pkgs" "$STAGE"
# Keep downloaded/extracted conda packages under build/. The source cache is
# appended read-only so restricted builders can still reuse existing packages.
export CONDA_PKGS_DIRS="$BUILD_ROOT/conda-pkgs:$CONDA_ROOT/pkgs"
if [ "$OFFLINE_BUILD" = "1" ]; then
  mapfile -t OFFLINE_CONDA_ARCHIVES < <("$SOURCE_PYTHON" - "$CONDA_ROOT/pkgs" <<'PY'
import json
import sys
from pathlib import Path

cache = Path(sys.argv[1]).resolve()
metadata_dir = Path(sys.prefix) / "conda-meta"
for metadata_path in sorted(metadata_dir.glob("*.json")):
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    filename = metadata.get("fn")
    candidates = [cache / filename] if filename else []
    stem = f"{metadata['name']}-{metadata['version']}-{metadata['build']}"
    candidates.extend(cache.glob(f"{stem}*.conda"))
    candidates.extend(cache.glob(f"{stem}*.tar.bz2"))
    archive = next((path for path in candidates if path.is_file()), None)
    if archive is None:
        raise FileNotFoundError(f"No cached archive for {stem}")
    print(archive)
PY
  )
  if [ "${#OFFLINE_CONDA_ARCHIVES[@]}" -eq 0 ]; then
    echo "[error] SOURCE_ENV 没有可复用的 conda 基础包" >&2
    exit 1
  fi
  "$CONDA_BIN" create -y -p "$BUILD_ENV" --offline "${OFFLINE_CONDA_ARCHIVES[@]}"
else
  "$CONDA_BIN" create -y -c conda-forge -p "$BUILD_ENV" \
    "python=$PYTHON_VERSION" pip conda-pack
fi

echo "[3/9] 安装 SONIC 仿真运行依赖"
if [ "$OFFLINE_BUILD" = "1" ]; then
  "$SOURCE_PYTHON" - "$BUILD_ENV" <<'PY'
import importlib.metadata
import os
import shutil
import sys
from collections import deque
from pathlib import Path

from packaging.markers import default_environment
from packaging.requirements import Requirement
from packaging.utils import canonicalize_name

destination_prefix = Path(sys.argv[1]).resolve()
source_prefix = Path(sys.prefix).resolve()
roots = ["numpy", "scipy", "mujoco", "PyYAML", "rich-click"]
distributions = {
    canonicalize_name(dist.metadata["Name"]): dist
    for dist in importlib.metadata.distributions()
    if dist.metadata["Name"]
}

requested_extras: dict[str, set[str]] = {}
processed_extras: dict[str, set[str]] = {}
pending = deque(Requirement(root) for root in roots)
selected: dict[str, importlib.metadata.Distribution] = {}
environment = default_environment()

while pending:
    requirement = pending.popleft()
    name = canonicalize_name(requirement.name)
    if name not in distributions:
        raise RuntimeError(f"SOURCE_ENV is missing required package: {requirement.name}")
    extras = requested_extras.setdefault(name, set())
    extras.update(requirement.extras)
    if name in processed_extras and extras <= processed_extras[name]:
        continue
    processed_extras[name] = set(extras)
    dist = distributions[name]
    selected[name] = dist

    marker_extras = extras or {""}
    for raw_child in dist.requires or []:
        child = Requirement(raw_child)
        if child.marker and not any(
            child.marker.evaluate({**environment, "extra": extra})
            for extra in marker_extras
        ):
            continue
        pending.append(child)

for name, dist in sorted(selected.items()):
    copied = 0
    for entry in dist.files or []:
        source = Path(os.path.abspath(dist.locate_file(entry)))
        try:
            relative = source.relative_to(source_prefix)
        except ValueError as error:
            raise RuntimeError(f"Package file escapes SOURCE_ENV: {source}") from error
        if not source.is_file():
            continue
        destination = destination_prefix / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied += 1
    print(f"  copied {dist.metadata['Name']}=={dist.version} ({copied} files)")
PY
else
  "$BUILD_ENV/bin/python" -m pip install --no-cache-dir \
    "numpy==$NUMPY_VERSION" \
    "scipy==$SCIPY_VERSION" \
    "mujoco==$MUJOCO_VERSION" \
    "PyYAML==$PYYAML_VERSION" \
    "rich-click==1.9.8"
fi

# Copy the ABI-matched binding without installing unitree_sdk2py: its setup.py
# declares OpenCV even though this simulator path never imports it.
BUILD_SITE="$("$BUILD_ENV/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
CYCLONEDDS_DIST=("$SOURCE_SITE"/cyclonedds-"$CYCLONEDDS_VERSION".dist-info)
if [ ! -d "$SOURCE_SITE/cyclonedds" ] || [ "${#CYCLONEDDS_DIST[@]}" -ne 1 ] \
  || [ ! -d "${CYCLONEDDS_DIST[0]}" ]; then
  echo "[error] '$SOURCE_ENV' 中的 CycloneDDS 安装不完整" >&2
  exit 1
fi
rsync -a --exclude='__pycache__' --exclude='*.pyc' \
  "$SOURCE_SITE/cyclonedds/" "$BUILD_SITE/cyclonedds/"
rsync -a "${CYCLONEDDS_DIST[0]}/" \
  "$BUILD_SITE/$(basename "${CYCLONEDDS_DIST[0]}")/"
cat > "$BUILD_SITE/cyclonedds/__library__.py" <<'PY'
import os
from pathlib import Path

in_wheel = False
library_path = Path(os.environ.get("CYCLONEDDS_HOME", "")) / "lib" / "libddsc.so"
PY
"$BUILD_ENV/bin/python" -m pip check

echo "[4/9] 复制最小 SONIC 源码和 Unitree DDS 消息定义"
GMR_STAGE="$STAGE/gmr"
mkdir -p "$GMR_STAGE/scripts" "$GMR_STAGE/sonic_sim/lib" \
  "$GMR_STAGE/sonic_sim/wbc_configs" \
  "$GMR_STAGE/third_party/unitree_sdk2_python/unitree_sdk2py"

find "$GMR_ROOT/sonic_sim" -maxdepth 1 -type f -name '*.py' \
  -exec cp -p {} "$GMR_STAGE/sonic_sim/" \;
cp -p "$GMR_ROOT/sonic_sim/lib/libddsc.so" "$GMR_STAGE/sonic_sim/lib/"
cp -p "$GMR_ROOT/sonic_sim/wbc_configs/g1_29dof_sonic_model12.yaml" \
  "$GMR_STAGE/sonic_sim/wbc_configs/"
cp -p "$GMR_ROOT/scripts/run_sonic_sim.py" "$GMR_STAGE/scripts/"

SDK_SOURCE="$GMR_ROOT/third_party/unitree_sdk2_python/unitree_sdk2py"
SDK_STAGE="$GMR_STAGE/third_party/unitree_sdk2_python/unitree_sdk2py"
cat > "$SDK_STAGE/__init__.py" <<'PY'
"""Minimal Unitree SDK surface required by the SONIC simulator."""

from . import core, idl, utils

__all__ = ["core", "idl", "utils"]
PY
for sdk_dir in core idl; do
  rsync -a --exclude='__pycache__' --exclude='*.pyc' \
    "$SDK_SOURCE/$sdk_dir/" "$SDK_STAGE/$sdk_dir/"
done
mkdir -p "$SDK_STAGE/utils"
for sdk_file in __init__.py singleton.py bqueue.py; do
  cp -p "$SDK_SOURCE/utils/$sdk_file" "$SDK_STAGE/utils/"
done

# Keep the licenses next to the redistributed sources.
cp -p "$GMR_ROOT/LICENSE" "$GMR_STAGE/LICENSE"
cp -p "$GMR_ROOT/third_party/unitree_sdk2_python/LICENSE" \
  "$GMR_STAGE/third_party/unitree_sdk2_python/LICENSE"

echo "[5/9] 解析 MJCF 并复制实际引用的模型资源"
"$BUILD_ENV/bin/python" - "$GMR_ROOT" "$GMR_STAGE" <<'PY'
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

source_root = Path(sys.argv[1]).resolve()
stage_root = Path(sys.argv[2]).resolve()
sonic_source = source_root / "sonic_sim"
sonic_stage = stage_root / "sonic_sim"
config_path = sonic_source / "wbc_configs" / "g1_29dof_sonic_model12.yaml"

with config_path.open(encoding="utf-8") as config_file:
    scene_rel = Path(yaml.safe_load(config_file)["ROBOT_SCENE"])

pending = [sonic_source / scene_rel]
copied_xml = set()

def copy_resource(source: Path) -> None:
    source = source.resolve()
    try:
        relative = source.relative_to(sonic_source)
    except ValueError as error:
        raise RuntimeError(f"Resource escapes sonic_sim: {source}") from error
    if not source.is_file():
        raise FileNotFoundError(source)
    destination = sonic_stage / relative
    destination.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, destination)

while pending:
    xml_path = pending.pop().resolve()
    if xml_path in copied_xml:
        continue
    copied_xml.add(xml_path)
    copy_resource(xml_path)

    root = ET.parse(xml_path).getroot()
    compiler = root.find("compiler")
    mesh_dir = compiler.get("meshdir", "") if compiler is not None else ""
    texture_dir = compiler.get("texturedir", "") if compiler is not None else ""

    for include in root.iter("include"):
        include_path = include.get("file")
        if include_path:
            pending.append(xml_path.parent / include_path)

    for tag, resource_dir in (("mesh", mesh_dir), ("texture", texture_dir), ("hfield", "")):
        for element in root.iter(tag):
            resource_path = element.get("file")
            if resource_path:
                copy_resource(xml_path.parent / resource_dir / resource_path)

print(f"  copied {len(copied_xml)} MJCF files and their referenced resources")
PY

echo "[6/9] 验证最小环境、DDS 绑定和 MuJoCo 模型"
export CYCLONEDDS_HOME="$GMR_STAGE/sonic_sim"
export LD_LIBRARY_PATH="$GMR_STAGE/sonic_sim/lib:$BUILD_ENV/lib:${LD_LIBRARY_PATH:-}"
export PYTHONPATH="$GMR_STAGE:$GMR_STAGE/third_party/unitree_sdk2_python"
"$BUILD_ENV/bin/python" - "$GMR_STAGE" <<'PY'
import ctypes
import importlib.metadata
import importlib.util
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
dds = root / "sonic_sim" / "lib" / "libddsc.so"
ctypes.CDLL(str(dds), mode=ctypes.RTLD_GLOBAL)

import cyclonedds
import mujoco
import numpy
import scipy
import yaml
from sonic_sim.configs import SimLoopConfig
from sonic_sim.simulator_factory import SimulatorFactory
import unitree_sdk2py
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

if set(unitree_sdk2py.__all__) != {"core", "idl", "utils"}:
    raise RuntimeError(f"Unexpected minimal Unitree SDK exports: {unitree_sdk2py.__all__}")

config = SimLoopConfig()
wbc_config = config.load_wbc_yaml()
model_path = root / "sonic_sim" / wbc_config["ROBOT_SCENE"]
model = mujoco.MjModel.from_xml_path(str(model_path))

expected = {
    "cyclonedds": "0.10.2",
    "mujoco": "3.9.0",
    "numpy": "2.4.6",
    "scipy": "1.17.1",
    "PyYAML": "6.0.3",
}
for package, version in expected.items():
    actual = importlib.metadata.version(package)
    if actual != version:
        raise RuntimeError(f"{package}: expected {version}, got {actual}")

for forbidden in ("torch", "triton", "PySide6", "cv2", "smplx"):
    if importlib.util.find_spec(forbidden) is not None:
        raise RuntimeError(f"Unexpected package in minimal environment: {forbidden}")

print(
    f"  verified Python {sys.version_info.major}.{sys.version_info.minor}, "
    f"{model.nbody} bodies, {model.njnt} joints, {model.ngeom} geoms"
)
PY

MAX_ENV_GLIBC="$(
  while IFS= read -r -d '' candidate; do
    if file -b "$candidate" | grep -q 'ELF'; then
      objdump -T "$candidate" 2>/dev/null | grep -oE 'GLIBC_[0-9.]+' || true
    fi
  done < <(find "$BUILD_ENV" -type f -print0)
  objdump -T "$GMR_STAGE/sonic_sim/lib/libddsc.so" 2>/dev/null \
    | grep -oE 'GLIBC_[0-9.]+' || true
)"
MAX_ENV_GLIBC="$(printf '%s\n' "$MAX_ENV_GLIBC" | sort -V | tail -n 1)"
if [ -n "$MAX_ENV_GLIBC" ] \
  && [ "$(printf '%s\n' "GLIBC_$TARGET_GLIBC" "$MAX_ENV_GLIBC" | sort -V | tail -n 1)" != "GLIBC_$TARGET_GLIBC" ]; then
  echo "[error] 最小环境需要 $MAX_ENV_GLIBC，目标 Ubuntu 要求不满足" >&2
  exit 1
fi
echo "  verified environment ABI <= GLIBC_${TARGET_GLIBC} (max ${MAX_ENV_GLIBC:-unknown})"

echo "[7/9] 打包并解压可重定位环境"
"$BUILD_ENV/bin/conda-pack" -p "$BUILD_ENV" \
  -o "$STAGE/env.tar.gz" --force --ignore-missing-files \
  --exclude='*.pyc' \
  --exclude='*/__pycache__/*' \
  --exclude='include/*' \
  --exclude='lib/*.a' \
  --exclude='lib/python*/test/*' \
  --exclude='lib/python*/site-packages/*/tests/*' \
  --exclude='share/doc/*' \
  --exclude='share/man/*'
mkdir -p "$STAGE/env"
tar -xzf "$STAGE/env.tar.gz" -C "$STAGE/env"
rm -f "$STAGE/env.tar.gz"

cp -p "$GMR_ROOT/scripts/run_sonic_sim.sh" "$STAGE/run_sonic_sim.sh"
chmod +x "$STAGE/run_sonic_sim.sh"
cp -p "$GMR_ROOT/SONIC_SIM_USAGE.md" "$STAGE/README.md"
cp -p "$GMR_ROOT/SONIC_SIM_USAGE.zh-CN.md" "$STAGE/README.zh-CN.md"

"$BUILD_ENV/bin/python" -m pip freeze > "$STAGE/PACKAGE_DEPENDENCIES.txt"
cat > "$STAGE/PACKAGE_INFO.txt" <<EOF
SONIC MuJoCo simulator
Target: Ubuntu 20.04 or newer, x86_64, glibc $TARGET_GLIBC+
Python: $PYTHON_VERSION
Source revision: $(git -C "$GMR_ROOT" rev-parse HEAD 2>/dev/null || printf unknown)
Build host: $OS_PRETTY_NAME
Build date: $(date -Iseconds)

Run with viewer:
  ./run_sonic_sim.sh

Run without viewer:
  ./run_sonic_sim.sh --no-onscreen

The C++ gear_sonic_deploy process is not included in this archive.
See README.md for English usage and README.zh-CN.md for Chinese usage.
EOF

echo "[8/9] 检查产物中没有大型非仿真依赖"
if [ ! -s "$STAGE/README.md" ] \
  || ! grep -q '^# SONIC MuJoCo Simulator Package$' "$STAGE/README.md" \
  || grep -q '^## 中文说明' "$STAGE/README.md"; then
  echo "[error] 产物缺少独立的英文使用说明" >&2
  exit 1
fi
if [ ! -s "$STAGE/README.zh-CN.md" ] \
  || ! grep -q '^# SONIC MuJoCo 仿真环境包$' "$STAGE/README.zh-CN.md" \
  || grep -q '^## English Guide' "$STAGE/README.zh-CN.md"; then
  echo "[error] 产物缺少独立的中文使用说明" >&2
  exit 1
fi
for forbidden_path in \
  "$STAGE/env/lib/python$PYTHON_VERSION/site-packages/torch" \
  "$STAGE/env/lib/python$PYTHON_VERSION/site-packages/triton" \
  "$STAGE/env/lib/python$PYTHON_VERSION/site-packages/nvidia" \
  "$STAGE/env/lib/python$PYTHON_VERSION/site-packages/PySide6" \
  "$STAGE/env/lib/python$PYTHON_VERSION/site-packages/cv2" \
  "$STAGE/gmr/assets" \
  "$STAGE/gmr/teleop_studio" \
  "$STAGE/gmr/general_motion_retargeting"; do
  if [ -e "$forbidden_path" ]; then
    echo "[error] 产物包含非仿真依赖：$forbidden_path" >&2
    exit 1
  fi
done

echo "[9/9] 生成分发包 dist/$PKG_NAME.tar.gz"
ARCHIVE="$GMR_ROOT/dist/$PKG_NAME.tar.gz"
tar -I 'gzip -9' -cf "$ARCHIVE" -C "$STAGE" .
(
  cd "$(dirname "$ARCHIVE")"
  sha256sum "$(basename "$ARCHIVE")" > "$(basename "$ARCHIVE").sha256"
)

echo "  environment: $(du -sh "$STAGE/env" | cut -f1)"
echo "  simulator:   $(du -sh "$STAGE/gmr" | cut -f1)"
echo "  archive:     $(du -sh "$ARCHIVE" | cut -f1)"
echo "完成：$ARCHIVE"
echo "校验：$ARCHIVE.sha256"

if [ "$KEEP_BUILD_ENV" != "1" ]; then
  rm -rf "$BUILD_ROOT"
fi
