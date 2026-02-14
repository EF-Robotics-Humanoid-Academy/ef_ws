"""Convert text to WAV using Google Text-to-Speech (gTTS).

Outputs mono 16kHz 16-bit WAV (compatible with the robot's AudioClient).
Requires: pip install gTTS
Requires: ffmpeg (for MP3 -> WAV conversion)
"""
import argparse
import os
import subprocess
import tempfile

from gtts import gTTS


def main():
    parser = argparse.ArgumentParser(description="Convert text to WAV file.")
    parser.add_argument("text", help="text to speak")
    parser.add_argument("-o", "--output", default="output.wav", help="output WAV path")
    parser.add_argument("--lang", default="en", help="language code (default: en)")
    args = parser.parse_args()

    tts = gTTS(text=args.text, lang=args.lang)

    fd, tmp_mp3 = tempfile.mkstemp(suffix=".mp3")
    os.close(fd)
    try:
        tts.save(tmp_mp3)
        subprocess.run(
            ["ffmpeg", "-y", "-i", tmp_mp3, "-ac", "1", "-ar", "16000", "-f", "wav", args.output],
            check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    finally:
        os.remove(tmp_mp3)

    print(f"Saved: {args.output}")


if __name__ == "__main__":
    main()
