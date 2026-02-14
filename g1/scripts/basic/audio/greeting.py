#!/usr/bin/env python3
"""Play greeting.wav locally or on the robot."""
from __future__ import annotations

import argparse
import os
import subprocess
import wave
import tempfile
import sys
from typing import Optional


def _find_player() -> list[str] | None:
    for cmd in ("aplay", "paplay", "ffplay"):
        if subprocess.call(["/usr/bin/env", "which", cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
            return [cmd]
    return None


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


def _parse_level(value: str) -> int:
    try:
        level = int(value)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("volume must be an integer 0-100") from exc
    if not 0 <= level <= 100:
        raise argparse.ArgumentTypeError("volume must be in range 0-100")
    return level


def _set_volume(level: int, iface: Optional[str]) -> None:
    _init_channel(iface)
    AudioClient = _load_audio_client()
    client = AudioClient()
    client.SetTimeout(3.0)
    client.Init()
    code = client.SetVolume(level)
    if code != 0:
        raise SystemExit(f"SetVolume failed: code={code}")


def _play_on_robot(wav_path: str, iface: Optional[str], volume: Optional[int]) -> None:
    _init_channel(iface)
    AudioClient = _load_audio_client()
    client = AudioClient()
    client.SetTimeout(5.0)
    client.Init()

    if volume is not None:
        code = client.SetVolume(volume)
        if code != 0:
            raise SystemExit(f"SetVolume failed: code={code}")

    tmp_path = None
    with wave.open(wav_path, "rb") as wf:
        channels = wf.getnchannels()
        rate = wf.getframerate()
        width = wf.getsampwidth()
        if channels != 1 or rate != 16000 or width != 2:
            player = _find_player()
            if not player:
                raise SystemExit(
                    "WAV must be mono 16-bit PCM at 16kHz for robot playback, "
                    "and no audio converter (ffmpeg) was found."
                )
            # Try ffmpeg conversion if available.
            if player[0] != "ffplay":
                try:
                    subprocess.run(["/usr/bin/env", "which", "ffmpeg"], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                except Exception:
                    raise SystemExit(
                        "WAV must be mono 16-bit PCM at 16kHz. Install ffmpeg for auto-convert."
                    )
                tmp_fd, tmp_path = tempfile.mkstemp(suffix=".wav")
                os.close(tmp_fd)
                subprocess.run(
                    ["ffmpeg", "-y", "-i", wav_path, "-ac", "1", "-ar", "16000", "-f", "wav", tmp_path],
                    check=True,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                wav_path = tmp_path
            # reopen converted file
            with wave.open(wav_path, "rb") as wf2:
                pcm = wf2.readframes(wf2.getnframes())
        else:
            pcm = wf.readframes(wf.getnframes())

    app_name = "greeting"
    stream_id = "greeting-1"
    code, data = client.PlayStream(app_name, stream_id, pcm)
    if code != 0:
        raise SystemExit(f"PlayStream failed: code={code}, data={data}")
    if tmp_path:
        try:
            os.remove(tmp_path)
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description="Play greeting.wav.")
    parser.add_argument("--file", default="greeting.wav", help="path to wav file")
    parser.add_argument("--volume", type=_parse_level, default=None, help="set robot speaker volume (0-100)")
    parser.add_argument("--iface", default=None, help="network interface for DDS (e.g. eth0)")
    parser.add_argument("--robot", action="store_true", help="play on robot via AudioClient")
    args = parser.parse_args()

    wav_path = args.file
    if not os.path.isabs(wav_path):
        wav_path = os.path.join(os.path.dirname(__file__), wav_path)

    if not os.path.exists(wav_path):
        print(f"Missing wav file: {wav_path}")
        sys.exit(1)

    if args.robot:
        _play_on_robot(wav_path, args.iface, args.volume)
        return

    player = _find_player()
    if not player:
        print("No audio player found. Install aplay/paplay/ffplay.")
        sys.exit(2)

    cmd = player + [wav_path]
    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as exc:
        print(f"Audio player failed: {exc}")
        sys.exit(exc.returncode)


if __name__ == "__main__":
    main()
