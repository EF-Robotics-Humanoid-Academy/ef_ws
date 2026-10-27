#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/platform.sh"
input_type="zmq"
zmq_host="localhost"
zmq_port="5556"
zmq_topic="pose"
target="real"
robot_interface="${ROBOT_INTERFACE:-}"
model_profile="release"
zmq_conflate=false

usage() {
    echo "Usage: $0 [--input-type zmq] --zmq-host HOST [--zmq-port PORT] [--zmq-topic pose] [--zmq-conflate] sim|real|INTERFACE"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --input-type)
            input_type="$2"
            shift 2
            ;;
        --zmq-host)
            zmq_host="$2"
            shift 2
            ;;
        --zmq-port)
            zmq_port="$2"
            shift 2
            ;;
        --zmq-topic)
            zmq_topic="$2"
            shift 2
            ;;
        --zmq-conflate)
            zmq_conflate=true
            shift
            ;;
        --cp|--checkpoint)
            case "$2" in
                policy/release/model|*/policy/release/model)
                    model_profile="release"
                    ;;
                policy/low_latency/model|*/policy/low_latency/model)
                    model_profile="low_latency"
                    ;;
                *)
                    echo "Error: unsupported model checkpoint '$2'." >&2
                    exit 2
                    ;;
            esac
            shift 2
            ;;
        --obs-config)
            # The selected profile always uses the matching packaged config.
            shift 2
            ;;
        --output-type)
            if [[ "$2" != "zmq" && "$2" != "all" ]]; then
                echo "Error: only ZMQ output is available in the onboard package." >&2
                exit 2
            fi
            shift 2
            ;;
        real|sim)
            target="$1"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        --*)
            echo "Error: unsupported onboard option '$1'." >&2
            exit 2
            ;;
        *)
            target="interface"
            robot_interface="$1"
            shift
            ;;
    esac
done

if [[ "$input_type" != "zmq" ]]; then
    echo "Error: this package only supports '--input-type zmq'." >&2
    exit 2
fi
if [[ "$zmq_topic" != "pose" ]]; then
    echo "Error: the GMR Moretion bridge publishes the required topic 'pose'." >&2
    exit 2
fi

if [[ "$target" == "sim" ]]; then
    if [[ "$(package_arch "$SCRIPT_DIR")" != "x86_64" ]]; then
        echo "Error: MuJoCo simulation is supported on Ubuntu x86_64 only." >&2
        exit 2
    fi
    robot_interface="lo"
elif [[ -z "$robot_interface" ]]; then
    robot_interface="$(ip -o -4 addr show | awk '$4 ~ /^192[.]168[.]123[.]/ {print $2; exit}')"
fi
if [[ -z "$robot_interface" ]]; then
    echo "Error: no 192.168.123.x Unitree DDS interface found; set ROBOT_INTERFACE." >&2
    exit 1
fi

echo "G1-EDU Moretion/GMR deployment"
echo "  Runtime target: $target"
echo "  DDS interface: $robot_interface"
echo "  GMR publisher: tcp://$zmq_host:$zmq_port"
echo "  Input topic:   $zmq_topic"
echo "  Model profile: $model_profile"
if [[ "$target" == "real" || "$target" == "interface" ]]; then
    echo "WARNING: this starts physical robot control. Keep the robot suspended."
else
    echo "Simulation mode: ensure an external Unitree DDS simulator is already running."
fi
read -r -p "Proceed with deployment? [Y/n]: " confirmation
if [[ "$confirmation" =~ ^[Nn]$ ]]; then
    echo "Cancelled."
    exit 0
fi

conflate_arg=""
if [[ "$zmq_conflate" == true ]]; then
    conflate_arg="--zmq-conflate"
fi

runtime_target="$target"
if [[ "$target" == "interface" ]]; then
    runtime_target="real"
fi

G1_DEPLOY_CONFIRMED=1 exec "$SCRIPT_DIR/run_onboard.sh" \
    "$robot_interface" "$zmq_host" "$zmq_port" "$model_profile" "$runtime_target" "$conflate_arg"
