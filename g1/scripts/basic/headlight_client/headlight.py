#!/usr/bin/env python3
"""Set the G1 headlight (RGB light strip) via Unitree AudioClient."""
from __future__ import annotations

import argparse
import re
import sys
import time
from typing import Optional


def _load_audio_client():
    try:
        from unitree_sdk2py.g1.audio.g1_audio_client import AudioClient  # type: ignore

        return AudioClient
    except Exception as exc:
        raise SystemExit(
            "unitree_sdk2py AudioClient is not available. Install unitree_sdk2_python and ensure AudioClient exists."
        ) from exc


def _init_channel(iface: Optional[str]) -> None:
    try:
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize  # type: ignore
    except Exception as exc:
        raise SystemExit(
            "unitree_sdk2py is not installed. Install it with:\n"
            "  pip install -e <path-to-unitree_sdk2_python>"
        ) from exc

    if iface:
        ChannelFactoryInitialize(0, iface)
    else:
        ChannelFactoryInitialize(0)


def _parse_intensity(value: str) -> int:
    try:
        level = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("intensity must be an integer 0-100") from exc
    if not 0 <= level <= 100:
        raise argparse.ArgumentTypeError("intensity must be in range 0-100")
    return level


_NAMED_COLORS = {
    "white": (255, 255, 255),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "yellow": (255, 255, 0),
    "cyan": (0, 255, 255),
    "magenta": (255, 0, 255),
    "orange": (255, 165, 0),
    "purple": (128, 0, 128),
    "pink": (255, 105, 180),
}


def _parse_color(value: str) -> tuple[int, int, int]:
    lowered = value.strip().lower()
    if lowered in _NAMED_COLORS:
        return _NAMED_COLORS[lowered]
    if re.fullmatch(r"#?[0-9a-fA-F]{6}", lowered):
        hexval = lowered.lstrip("#")
        return (int(hexval[0:2], 16), int(hexval[2:4], 16), int(hexval[4:6], 16))
    if re.fullmatch(r"\\d{1,3},\\d{1,3},\\d{1,3}", lowered):
        parts = [int(p) for p in lowered.split(",")]
        if all(0 <= p <= 255 for p in parts):
            return (parts[0], parts[1], parts[2])
    raise argparse.ArgumentTypeError("color must be a name, #RRGGBB, or R,G,B (0-255)")


def _scale_color(rgb: tuple[int, int, int], intensity: int) -> tuple[int, int, int]:
    if intensity >= 100:
        return rgb
    scale = intensity / 100.0
    return (int(rgb[0] * scale), int(rgb[1] * scale), int(rgb[2] * scale))


def main() -> int:
    parser = argparse.ArgumentParser(description="Set headlight color/intensity using AudioClient.")
    parser.add_argument("--iface", default=None, help="network interface for DDS (e.g. eth0)")
    parser.add_argument("--intensity", type=_parse_intensity, default=100, help="brightness level 0-100")
    parser.add_argument("--color", type=_parse_color, default=_NAMED_COLORS["white"], help="headlight color")
    args = parser.parse_args()

    _init_channel(args.iface)
    AudioClient = _load_audio_client()
    client = AudioClient()
    client.SetTimeout(3.0)
    client.Init()

    r, g, b = _scale_color(args.color, args.intensity)
    code = client.LedControl(r, g, b)
    if code != 0:
        print(f"LedControl failed: code={code}")
        return 1

    print(f"Headlight set to RGB({r},{g},{b}) at {args.intensity}% intensity.")
    # LED control requires >200ms between calls
    time.sleep(0.25)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
