#!/usr/bin/env python3
"""Record a WAV file with Enter-to-start and Enter-to-stop controls (Windows/macOS/Linux)."""

from __future__ import annotations

import argparse
import os
import queue
import sys

# pip install sounddevice soundfile
import sounddevice as sd
import soundfile as sf


def record_wav(out_path: str, rate: int, channels: int, subtype: str) -> int:
    q: queue.Queue = queue.Queue()

    def callback(indata, frames, time_info, status):
        if status:
            # Print warnings/errors from the audio backend
            print(status, file=sys.stderr)
        q.put(indata.copy())

    print("Recording... Press Enter to stop recording...")

    try:
        with sf.SoundFile(out_path, mode="x", samplerate=rate, channels=channels, subtype=subtype) as f:
            with sd.InputStream(samplerate=rate, channels=channels, callback=callback):
                input()  # blocks until Enter
                # Drain any remaining buffered chunks
                while not q.empty():
                    f.write(q.get_nowait())

        return 0

    except FileExistsError:
        print(f"Output already exists: {out_path}")
        return 2
    except sd.PortAudioError as e:
        print(f"Audio device error: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"Recording failed: {e}", file=sys.stderr)
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Record a WAV file with Enter-to-start and Enter-to-stop.")
    parser.add_argument("outfile", help="output WAV filename")
    parser.add_argument("--rate", type=int, default=16000, help="sample rate (Hz)")
    parser.add_argument("--channels", type=int, default=1, help="number of channels")
    # WAV subtypes supported by soundfile/libsndfile (common ones: PCM_16, PCM_24, PCM_32, FLOAT)
    parser.add_argument("--subtype", default="PCM_16", help="WAV subtype (default: PCM_16)")
    args = parser.parse_args()

    out_path = args.outfile
    if not out_path.lower().endswith(".wav"):
        out_path += ".wav"

    if os.path.exists(out_path):
        print(f"Output already exists: {out_path}")
        return 2

    print("Press Enter to start recording...")
    input()

    return record_wav(out_path, args.rate, args.channels, args.subtype)


if __name__ == "__main__":
    raise SystemExit(main())
