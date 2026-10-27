#!/usr/bin/env bash
# Teleop launcher with the safety layer (smoothing, dead-man, watchdog, limits).
# The legacy deploy.sh / run_onboard.sh path is unchanged and does NOT use it.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
TARGET_ARCH="$(package_arch "$SCRIPT_DIR")"
require_matching_package_arch "$SCRIPT_DIR"

DEFAULT_STREAMER="10.34.0.135"
STAMP="$SCRIPT_DIR/logs/teleop_safety/VALIDATED"

usage() {
    cat <<EOF
Usage:
  $0 selftest [--model release|low_latency] [--config FILE]
      Offline unit tests + automated dry-run on loopback with a synthetic
      mocap stream and injected faults. Writes the validation stamp on PASS.

  $0 dry-run [--streamer HOST] [--port 5556] [--model M] [--iface lo|eth0] [--config FILE] [--set key=value ...]
      Full pipeline against the live mocap streamer; NOTHING is sent to the
      robot. --iface lo (default) uses a synthetic robot state; --iface eth0
      reads the real robot state (shadow mode) but still sends nothing.

  $0 real [--streamer HOST] [--port 5556] [--model M] [--iface IFACE] [--config FILE]
      Physical robot. Requires a PASSing selftest for the same config file,
      binary and model, and typing ENABLE at the prompt.

Defaults: --streamer $DEFAULT_STREAMER --port 5556 --model release --config config/teleop_safety.conf
Keys in the controller terminal: ] start control | Enter mocap on/off | hold SPACE dead-man | O emergency stop
EOF
}

[[ $# -ge 1 ]] || { usage >&2; exit 2; }
command="$1"; shift

streamer="$DEFAULT_STREAMER"
port="5556"
model="release"
config="config/teleop_safety.conf"
iface=""
sets=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --streamer) streamer="$2"; shift 2 ;;
        --port) port="$2"; shift 2 ;;
        --model) model="$2"; shift 2 ;;
        --config) config="$2"; shift 2 ;;
        --iface) iface="$2"; shift 2 ;;
        --set) sets+=(--safety-set "$2"); shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Error: unknown option '$1'" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ "$model" != "release" && "$model" != "low_latency" ]]; then
    echo "Error: --model must be release or low_latency" >&2; exit 2
fi
if [[ ! "$port" =~ ^[0-9]+$ ]] || (( port < 1 || port > 65535 )); then
    echo "Error: invalid port '$port'" >&2; exit 2
fi
cd "$SCRIPT_DIR"
[[ -f "$config" ]] || { echo "Error: config '$config' not found" >&2; exit 2; }

binary="$SCRIPT_DIR/bin/g1_deploy_onnx_ref"
[[ -x "$binary" ]] || { echo "Error: $binary missing; run ./build_onboard.sh" >&2; exit 1; }

onnx_root="${onnxruntime_ROOT:-}"
if [[ -z "$onnx_root" && -d "$SCRIPT_DIR/thirdparty/onnxruntime" ]]; then onnx_root="$SCRIPT_DIR/thirdparty/onnxruntime"; fi
export LD_LIBRARY_PATH="$SCRIPT_DIR/thirdparty/unitree_sdk2/thirdparty/lib/$TARGET_ARCH:$onnx_root/lib:${TensorRT_ROOT:-/usr}/lib:${LD_LIBRARY_PATH:-}"

ts="$(date +%Y%m%d_%H%M%S)"
mkdir -p logs/teleop_safety

controller_args=(
    "policy/$model/model_decoder.onnx" reference/
    --obs-config "policy/$model/observation_config.yaml"
    --encoder-file "policy/$model/model_encoder.onnx"
    --input-type zmq --output-type zmq
    --zmq-host "$streamer" --zmq-port "$port" --zmq-topic pose --zmq-conflate
    --teleop-safety-config "$config"
)

case "$command" in
    selftest)
        echo "== Offline unit tests =="
        "$SCRIPT_DIR/tests/run_offline_tests.sh" | grep -E '^\[test\]|^    |FAIL|checks,'
        echo
        echo "== Automated dry-run (loopback, synthetic mocap, fault injection) =="
        exec python3 "$SCRIPT_DIR/tools/dry_run_selftest.py" --config "$config" --model "$model"
        ;;

    dry-run)
        iface="${iface:-lo}"
        [[ -d "/sys/class/net/$iface" ]] || { echo "Error: interface '$iface' does not exist" >&2; exit 1; }
        csv="logs/teleop_safety/dryrun_${ts}.csv"
        echo "DRY-RUN: nothing is sent to the robot. Streamer tcp://$streamer:$port  iface=$iface  CSV=$csv"
        exec "$binary" "$iface" "${controller_args[@]}" --dry-run \
            --safety-set "csv_log_path=$csv" "${sets[@]}"
        ;;

    real)
        if [[ ${#sets[@]} -gt 0 ]]; then
            echo "Error: --set is not allowed for real runs; edit $config and re-run the selftest." >&2; exit 2
        fi
        if [[ ! -f "$STAMP" ]]; then
            echo "Error: no dry-run validation found. Run: $0 selftest --model $model --config $config" >&2; exit 1
        fi
        cfg_abs="$(cd "$(dirname "$config")" && pwd)/$(basename "$config")"
        want_cfg="$(sha256sum "$cfg_abs" | cut -d' ' -f1)"
        want_bin="$(sha256sum "$binary" | cut -d' ' -f1)"
        python3 - "$STAMP" "$want_cfg" "$want_bin" "$model" <<'PY' || exit 1
import json, sys
stamp, cfg, binary, model = sys.argv[1:]
s = json.load(open(stamp))
problems = []
if s.get("config_sha256") != cfg: problems.append("config file changed since validation")
if s.get("binary_sha256") != binary: problems.append("controller binary changed since validation")
if s.get("model") != model: problems.append(f"validated with model '{s.get('model')}', not '{model}'")
if problems:
    print("Error: dry-run validation does not match: " + "; ".join(problems), file=sys.stderr)
    print("Re-run the selftest before physical deployment.", file=sys.stderr)
    sys.exit(1)
print(f"Dry-run validation OK ({s['timestamp']}, report {s['report']})")
PY
        if [[ -z "$iface" ]]; then
            iface="$(ip -o -4 addr show | awk '$4 ~ /^192[.]168[.]123[.]/ {print $2; exit}')"
        fi
        [[ -n "$iface" && -d "/sys/class/net/$iface" ]] || { echo "Error: no Unitree DDS interface (192.168.123.x); use --iface" >&2; exit 1; }
        csv="logs/teleop_safety/real_${ts}.csv"
        echo "PHYSICAL ROBOT  iface=$iface  streamer tcp://$streamer:$port  model=$model  CSV=$csv"
        echo "Checklist: robot suspended and unloaded | E-stop in hand | workspace clear | lowest speed settings"
        exec "$binary" "$iface" "${controller_args[@]}" --safety-set "csv_log_path=$csv"
        ;;

    *)
        usage >&2; exit 2 ;;
esac
