"""
Generate positive and negative WAV samples for wakeword training.

Uses two TTS sources for acoustic diversity:
  - Piper TTS  (neural, 904 speakers)
  - macOS `say` (system TTS, ~15 distinct voices — sounds completely different)

Also saves noise-augmented copies of every Piper sample to force the
embedding model to see more varied representations of each phrase.

Usage:
    python3 generate_training_samples.py
    python3 generate_training_samples.py --pos_n 500 --neg_n 2000
"""

from __future__ import annotations

import argparse
import random
import shutil
import subprocess
import wave
from math import gcd
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------------------
# Phrases
# ---------------------------------------------------------------------------

POSITIVE_PHRASES = [
    "hey wolfwagen",
    "hey wolf wagon",       # alternative pronunciation
    "hey wolf vagen",       # German-influenced variant
    "wolfwagen",
    "wolf wagon",           # shortened activation
]

NEGATIVE_PHRASES = [
    "what time is it",
    "turn on the lights",
    "play some music",
    "set a timer for five minutes",
    "what is the weather today",
    "navigate to the campus entrance",
    "how far is the nearest building",
    "stop right there",
    "slow down please",
    "increase your speed",
    "what is your current status",
    "report your location now",
    "activate patrol mode",
    "return to base immediately",
    "can you hear me",
    "system check complete",
    "all units stand by",
    "the package has been delivered",
    "please confirm your heading",
    "obstacle detected on the left",
    "battery level is at fifty percent",
    "connecting to the network",
    "mission accomplished",
    "standby for further instructions",
    "route recalculation in progress",
    "scanning the perimeter",
    "no anomalies detected",
    "maintaining current speed",
    "emergency stop activated",
    "resuming autonomous navigation",
]

# ---------------------------------------------------------------------------
# Piper TTS config
# ---------------------------------------------------------------------------
LENGTH_SCALES = [0.80, 0.90, 1.00, 1.10, 1.20]
NOISE_SCALES  = [0.30, 0.50, 0.70]
NOISE_W_VALS  = [0.80, 1.00, 1.20]

PIPER_MODEL_PATH  = Path(__file__).parent / "piper_models/en_US-libritts_r-medium.onnx"
PIPER_CONFIG_PATH = Path(__file__).parent / "piper_models/en_US-libritts_r-medium.onnx.json"

# ---------------------------------------------------------------------------
# macOS `say` voices  (natural-sounding English only)
# ---------------------------------------------------------------------------
SAY_VOICES = [
    "Alex", "Daniel", "Fred", "Kathy", "Samantha",
    "Reed", "Sandy", "Shelley", "Flo", "Eddy", "Rocko",
]

TARGET_RATE = 16000

# SNR levels for noise augmentation (dB) — lower = noisier
NOISE_SNR_LEVELS = [20.0, 12.0, 6.0]


# ---------------------------------------------------------------------------
# Audio utilities
# ---------------------------------------------------------------------------

def resample_to_16k(audio: np.ndarray, src_rate: int) -> np.ndarray:
    from scipy.signal import resample_poly
    g = gcd(src_rate, TARGET_RATE)
    return resample_poly(audio, TARGET_RATE // g, src_rate // g).astype(np.int16)


def save_wav(audio: np.ndarray, path: Path) -> None:
    with wave.open(str(path), "w") as wf:
        wf.setnchannels(1)
        wf.setsampwidth(2)
        wf.setframerate(TARGET_RATE)
        wf.writeframes(audio.tobytes())


def load_wav_int16(path: Path) -> np.ndarray:
    with wave.open(str(path), "r") as wf:
        raw = wf.readframes(wf.getnframes())
    return np.frombuffer(raw, dtype=np.int16)


def add_noise(audio: np.ndarray, snr_db: float) -> np.ndarray:
    """Add Gaussian white noise at the given SNR (dB)."""
    signal = audio.astype(np.float32)
    sig_power = np.mean(signal ** 2) + 1e-9
    noise_power = sig_power / (10 ** (snr_db / 10))
    noise = np.random.normal(0, np.sqrt(noise_power), len(signal)).astype(np.float32)
    return np.clip(signal + noise, -32768, 32767).astype(np.int16)


# ---------------------------------------------------------------------------
# Piper synthesis
# ---------------------------------------------------------------------------

def piper_synthesize(voice, phrase: str, speaker_id: int) -> np.ndarray:
    from piper.config import SynthesisConfig
    cfg = SynthesisConfig(
        speaker_id=speaker_id,
        length_scale=random.choice(LENGTH_SCALES),
        noise_scale=random.choice(NOISE_SCALES),
        noise_w_scale=random.choice(NOISE_W_VALS),
    )
    chunks = [c.audio_int16_array for c in voice.synthesize(phrase, syn_config=cfg)]
    return np.concatenate(chunks) if chunks else np.array([], dtype=np.int16)


def generate_piper(phrases: list[str], out_dir: Path, n_total: int,
                   voice, num_speakers: int, src_rate: int, label: str) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    n_per_phrase = max(1, n_total // len(phrases))
    count = 0

    for phrase in phrases:
        for i in range(n_per_phrase):
            spk = random.randint(0, num_speakers - 1)
            audio = piper_synthesize(voice, phrase, spk)
            if len(audio) == 0:
                continue
            if src_rate != TARGET_RATE:
                audio = resample_to_16k(audio, src_rate)

            tag = phrase.replace(" ", "_")
            clean_path = out_dir / f"{tag}_piper_{i:04d}_spk{spk:04d}.wav"
            save_wav(audio, clean_path)
            count += 1

            # Save noise-augmented copies for each SNR level
            for snr in NOISE_SNR_LEVELS:
                noisy = add_noise(audio, snr)
                noisy_path = out_dir / f"{tag}_piper_{i:04d}_spk{spk:04d}_snr{int(snr)}.wav"
                save_wav(noisy, noisy_path)
                count += 1

            if count % 200 == 0:
                print(f"  [{label}/piper] {count} samples...")

    return count


# ---------------------------------------------------------------------------
# macOS `say` synthesis
# ---------------------------------------------------------------------------

def say_available() -> bool:
    return shutil.which("say") is not None


def generate_say(phrases: list[str], out_dir: Path, n_per_phrase: int, label: str) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)
    count = 0

    for phrase in phrases:
        voices = random.choices(SAY_VOICES, k=n_per_phrase)
        for i, voice in enumerate(voices):
            out_path = out_dir / f"{phrase.replace(' ', '_')}_say_{i:04d}_{voice}.wav"
            result = subprocess.run(
                ["say", "-v", voice, "-o", str(out_path),
                 "--data-format=LEI16@16000", phrase],
                capture_output=True,
            )
            if result.returncode != 0 or not out_path.exists():
                continue

            audio = load_wav_int16(out_path)
            count += 1

            # Noise-augmented copies
            for snr in NOISE_SNR_LEVELS:
                noisy = add_noise(audio, snr)
                noisy_path = out_dir / f"{phrase.replace(' ', '_')}_say_{i:04d}_{voice}_snr{int(snr)}.wav"
                save_wav(noisy, noisy_path)
                count += 1

            if count % 100 == 0:
                print(f"  [{label}/say] {count} samples...")

    return count


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(pos_n: int, neg_n: int, out_base: Path) -> None:
    from piper import PiperVoice

    print("[Train] Loading Piper TTS model...")
    voice = PiperVoice.load(str(PIPER_MODEL_PATH), config_path=str(PIPER_CONFIG_PATH))
    num_speakers = voice.config.num_speakers or 1
    src_rate = voice.config.sample_rate
    print(f"[Train] Piper: {num_speakers} speakers @ {src_rate} Hz")

    use_say = say_available()
    if use_say:
        print(f"[Train] macOS `say` available — will use {len(SAY_VOICES)} system voices")
    else:
        print("[Train] macOS `say` not found — using Piper only")

    # --- positives ---
    print(f"\n[Train] Generating positive samples...")
    pos_dir = out_base / "positive"
    piper_pos = pos_n // 2 if use_say else pos_n
    say_pos_per_phrase = max(5, (pos_n // 2) // len(POSITIVE_PHRASES)) if use_say else 0

    n = generate_piper(POSITIVE_PHRASES, pos_dir, piper_pos, voice, num_speakers, src_rate, "positive")
    if use_say:
        n += generate_say(POSITIVE_PHRASES, pos_dir, say_pos_per_phrase, "positive")
    print(f"  [positive] Total: {len(list(pos_dir.glob('*.wav')))} WAV files")

    # --- negatives ---
    print(f"\n[Train] Generating negative samples...")
    neg_dir = out_base / "negative"
    piper_neg = neg_n // 2 if use_say else neg_n
    say_neg_per_phrase = max(5, (neg_n // 2) // len(NEGATIVE_PHRASES)) if use_say else 0

    n = generate_piper(NEGATIVE_PHRASES, neg_dir, piper_neg, voice, num_speakers, src_rate, "negative")
    if use_say:
        n += generate_say(NEGATIVE_PHRASES, neg_dir, say_neg_per_phrase, "negative")
    print(f"  [negative] Total: {len(list(neg_dir.glob('*.wav')))} WAV files")

    print(f"\n[Train] Done. Run:  python3 train_wakeword.py")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--pos_n", type=int, default=500,
                        help="Base positive samples from Piper (noise copies are added on top)")
    parser.add_argument("--neg_n", type=int, default=2000,
                        help="Base negative samples from Piper (noise copies are added on top)")
    parser.add_argument("--out_dir", type=Path,
                        default=Path(__file__).parent / "training_data")
    args = parser.parse_args()

    # Wipe and regenerate for a clean run
    import shutil as _shutil
    if args.out_dir.exists():
        print(f"[Train] Removing old training data at {args.out_dir}...")
        _shutil.rmtree(args.out_dir)

    main(args.pos_n, args.neg_n, args.out_dir)
