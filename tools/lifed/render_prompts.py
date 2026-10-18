# -*- coding: utf-8 -*-
"""Record the human-voice prompts (prompts.py) for the robot: edge-tts Xiaoxiao -> 48 kHz mono wav,
through the level chain chosen for the HAT's speaker on 2026-10-09 (250 Hz high-pass, 4:1 above
-24 dB, peak to -1 dBFS, a little makeup gain into a tanh limit).

Needs the internet, edge-tts >= 7.2.8 (older ones get 403), ffmpeg, numpy, scipy, soundfile:
    python tools/lifed/render_prompts.py out_dir [--gain-db 3] [--only key ...]
then copy out_dir/*.wav to the robot's ~/lifed/voice/.
"""
from __future__ import annotations

import argparse
import asyncio
import os
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from prompts import DIGITS, PROMPTS  # noqa: E402

VOICE = "zh-CN-XiaoxiaoNeural"
RATE = 48000


async def tts(text: str, mp3: str):
    import edge_tts
    await edge_tts.Communicate(text, VOICE).save(mp3)


def decode(mp3: str) -> np.ndarray:
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", mp3, "-f", "s16le", "-ac", "1", "-ar", str(RATE), "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.int16).astype(np.float64) / 32768.0


def trim(x: np.ndarray, floor_db: float = -40.0) -> np.ndarray:
    """Drop the silence edge-tts puts around a single character."""
    loud = np.nonzero(np.abs(x) > 10 ** (floor_db / 20))[0]
    return x[max(0, loud[0] - int(0.01 * RATE)):loud[-1] + int(0.03 * RATE)] if len(loud) else x


def level(x: np.ndarray, gain_db: float, lead_s: float = 0.08) -> np.ndarray:
    from scipy.signal import butter, sosfilt
    x = sosfilt(butter(2, 250, "highpass", fs=RATE, output="sos"), x)
    # 4:1 above -24 dB on a 10 ms RMS envelope
    win = int(0.010 * RATE)
    env = np.sqrt(np.convolve(x * x, np.ones(win) / win, mode="same") + 1e-12)
    env_db = 20 * np.log10(env)
    over = np.maximum(env_db + 24.0, 0.0)
    x = x * 10 ** (-(over * (1 - 1 / 4.0)) / 20)
    x = x / (np.max(np.abs(x)) + 1e-12) * 10 ** (-1 / 20)
    x = x * 10 ** (gain_db / 20)
    lim = 10 ** (-1 / 20)
    x = lim * np.tanh(x / lim)
    # 30 ms fades, 80 ms of silence ahead so aplay's start does not eat the first syllable
    f = int(0.030 * RATE)
    x[:f] *= np.linspace(0, 1, f)
    x[-f:] *= np.linspace(1, 0, f)
    return np.concatenate([np.zeros(int(lead_s * RATE)), x])


def main():
    import soundfile as sf
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("--gain-db", type=float, default=3.0)
    ap.add_argument("--only", nargs="*")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        for key, text in (PROMPTS | DIGITS).items():
            if a.only and key not in a.only:
                continue
            mp3 = os.path.join(tmp, key + ".mp3")
            asyncio.run(tts(text, mp3))
            x = decode(mp3)
            # a digit is one of a row: no silence ahead (lifed puts the gaps in)
            x = level(trim(x), a.gain_db, lead_s=0.0) if key in DIGITS else level(x, a.gain_db)
            path = os.path.join(a.out, key + ".wav")
            sf.write(path, (x * 32767).astype(np.int16), RATE, subtype="PCM_16")
            print(f"{key}: {len(x) / RATE:.1f} s, {len(text)} chars  {text}")


if __name__ == "__main__":
    main()
