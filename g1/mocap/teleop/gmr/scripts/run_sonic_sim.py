#!/usr/bin/env python3
"""Entry point for running a MuJoCo simulation loop with the G1 robot model.

Loads a YAML-based WBC config, initializes DDS channels, and launches the
MuJoCo simulator. The simulator receives motor commands from the C++ WBC
deployment (gear_sonic_deploy) over DDS and publishes simulated sensor
state back, so the WBC policy sees the sim as a real robot.

Usage::

    # In GMR conda environment:
    source ~/.holosoma_deps/miniconda3/bin/activate gmr
    python scripts/run_sonic_sim.py

    # With offscreen rendering disabled (default):
    python scripts/run_sonic_sim.py --no-onscreen

    # With custom interface:
    python scripts/run_sonic_sim.py --interface lo

Then in a separate terminal, launch the C++ deploy::

    cd /home/zpf/workspace/GR00T-WholeBodyControl/gear_sonic_deploy
    bash deploy.sh --input-type zmq \\
        --zmq-host localhost --zmq-port 5556 --zmq-topic pose sim
"""

import argparse
import ctypes
import os
import platform
import sys
from pathlib import Path
from typing import Dict

# Add project root to path so sonic_sim is importable
_PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_PROJECT_ROOT))

# Preload CycloneDDS 0.10.2 C library before importing cyclonedds/_clayer.
# Must match the C++ deploy's libddsc.so.0 version for DDS interoperability.
# On Windows, the pip-installed cyclonedds package already bundles ddsc.dll.
if platform.system() != "Windows":
    _DDS_LIB_PATH = str(_PROJECT_ROOT / "sonic_sim" / "lib" / "libddsc.so")
    try:
        ctypes.CDLL(_DDS_LIB_PATH, mode=ctypes.RTLD_GLOBAL)
    except OSError:
        # Fallback: rely on LD_LIBRARY_PATH if preload fails
        _DDS_LIB_DIR = str(_PROJECT_ROOT / "sonic_sim" / "lib")
        os.environ["LD_LIBRARY_PATH"] = _DDS_LIB_DIR + ":" + os.environ.get("LD_LIBRARY_PATH", "")

from sonic_sim.simulator_factory import SimulatorFactory, init_channel
from sonic_sim.configs import SimLoopConfig


def main(config: SimLoopConfig):
    wbc_config = config.load_wbc_yaml()
    wbc_config["ENV_NAME"] = config.env_name

    if config.enable_image_publish:
        assert (
            config.enable_offscreen
        ), "enable_offscreen must be True when enable_image_publish is True"

    # Initialize DDS channel
    init_channel(config=wbc_config)

    # Create simulator using factory
    sim = SimulatorFactory.create_simulator(
        config=wbc_config,
        env_name=config.env_name,
        onscreen=wbc_config.get("ENABLE_ONSCREEN", True),
        offscreen=wbc_config.get("ENABLE_OFFSCREEN", False),
        enable_image_publish=config.enable_image_publish,
    )

    if sim is None:
        print("Failed to create simulator.")
        return 1

    # Start simulator (blocking)
    SimulatorFactory.start_simulator(
        sim,
        as_thread=False,
        enable_image_publish=config.enable_image_publish,
        mp_start_method=config.mp_start_method,
        camera_port=config.camera_port,
    )

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SONIC MuJoCo simulation loop for G1 robot. "
        "Receives DDS motor commands from C++ WBC deploy and simulates physics.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # System Configuration
    sys_group = parser.add_argument_group("System Configuration")
    sys_group.add_argument(
        "--interface", type=parse_sim_interface, default="sim",
        help="DDS network interface: 'sim' (loopback), or an interface name/IP",
    )
    sys_group.add_argument(
        "--sim-frequency", type=int, default=200,
        help="Simulation frequency in Hz",
    )
    sys_group.add_argument(
        "--control-frequency", type=int, default=50,
        help="Control frequency in Hz",
    )

    # Rendering
    render_group = parser.add_argument_group("Rendering")
    render_group.add_argument(
        "--onscreen", action="store_true", default=True,
        help="Enable onscreen MuJoCo viewer (default: True)",
    )
    render_group.add_argument(
        "--no-onscreen", dest="onscreen", action="store_false",
        help="Disable onscreen viewer",
    )
    render_group.add_argument(
        "--offscreen", action="store_true", default=False,
        help="Enable offscreen rendering",
    )
    render_group.add_argument(
        "--enable-image-publish", action="store_true", default=False,
        help="Enable image publishing via ZMQ (requires --offscreen)",
    )
    render_group.add_argument(
        "--camera-port", type=int, default=5555,
        help="ZMQ port for camera image publishing",
    )
    render_group.add_argument(
        "--mp-start-method", type=str, default="spawn",
        help="Multiprocessing start method",
    )

    # Robot Configuration
    robot_group = parser.add_argument_group("Robot Configuration")
    robot_group.add_argument(
        "--enable-waist", action="store_true", default=True,
        help="Include waist joints",
    )
    robot_group.add_argument(
        "--with-hands", action="store_true", default=True,
        help="Enable hand functionality",
    )

    # WBC
    wbc_group = parser.add_argument_group("WBC Configuration")
    wbc_group.add_argument(
        "--wbc-version", type=str, default="sonic_model12",
        help="WBC version",
    )

    return parser


def parse_sim_interface(value: str) -> str:
    """Parse a simulation DDS interface and reject the real-robot shortcut.

    The simulator can use a concrete network interface or IP for a remote WBC,
    but ``real`` is intentionally not accepted here: that shared configuration
    shortcut marks the environment as real and applies sim-to-real tuning.
    """
    if value.lower() == "real":
        raise argparse.ArgumentTypeError(
            "'real' is not valid for MuJoCo simulation; use 'sim' or a network interface/IP"
        )
    return value


def args_to_config(args: argparse.Namespace) -> SimLoopConfig:
    """Convert argparse namespace to SimLoopConfig dataclass."""
    config = SimLoopConfig()
    config.interface = args.interface
    config.sim_frequency = args.sim_frequency
    config.control_frequency = args.control_frequency
    config.enable_onscreen = args.onscreen
    config.enable_offscreen = args.offscreen
    config.enable_image_publish = args.enable_image_publish
    config.camera_port = args.camera_port
    config.mp_start_method = args.mp_start_method
    config.enable_waist = args.enable_waist
    config.with_hands = args.with_hands
    config.wbc_version = args.wbc_version
    # Resolve the DDS interface, but keep simulation tuning even when a
    # concrete non-loopback interface is used for a remote WBC.
    config.__post_init__()
    config.env_type = "sim"
    return config


if __name__ == "__main__":
    parser = build_parser()
    args = parser.parse_args()
    config = args_to_config(args)
    sys.exit(main(config))
