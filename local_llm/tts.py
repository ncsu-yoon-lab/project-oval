"""
Text-to-Speech module using Piper TTS.
Runs on CPU, leaving GPU free for LLM and Whisper.
Outputs raw PCM audio that can be played directly via PyAudio.
"""

import io
import subprocess
import shutil
import wave
import numpy as np
from pathlib import Path

# Default Piper voice model (will be downloaded by setup.sh)
DEFAULT_PIPER_MODEL = "en_US-lessac-medium"
DEFAULT_PIPER_DATA_DIR = Path(__file__).parent / "piper_models"
DEFAULT_SAMPLE_RATE = 22050


class TextToSpeech:
    """Wraps the Piper TTS CLI for text-to-speech synthesis."""

    def __init__(
        self,
        model: str = DEFAULT_PIPER_MODEL,
        data_dir: Path = DEFAULT_PIPER_DATA_DIR,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ):
        self.model = model
        self.data_dir = data_dir
        self.sample_rate = sample_rate
        self.piper_bin = None

    def load(self):
        """Verify piper binary is available."""
        self.piper_bin = shutil.which("piper")
        if self.piper_bin is None:
            # Check local install
            local_piper = Path(__file__).parent / "piper" / "piper"
            if local_piper.exists():
                self.piper_bin = str(local_piper)
            else:
                raise FileNotFoundError(
                    "Piper TTS binary not found. Run setup.sh to install it."
                )

        model_path = self.data_dir / f"{self.model}.onnx"
        if not model_path.exists():
            raise FileNotFoundError(
                f"Piper model not found at {model_path}. Run setup.sh to download models."
            )

        print(f"[TTS] Piper ready: binary={self.piper_bin}, model={self.model}")

    def synthesize(self, text: str) -> np.ndarray:
        """
        Convert text to audio samples.

        Args:
            text: The text to speak.

        Returns:
            float32 numpy array of audio samples at self.sample_rate.
        """
        if not text or not text.strip():
            return np.array([], dtype=np.float32)

        model_path = self.data_dir / f"{self.model}.onnx"
        config_path = self.data_dir / f"{self.model}.onnx.json"

        cmd = [
            self.piper_bin,
            "--model", str(model_path),
            "--config", str(config_path),
            "--output-raw",
            "--quiet",
        ]

        result = subprocess.run(
            cmd,
            input=text.encode("utf-8"),
            capture_output=True,
            timeout=30,
        )

        if result.returncode != 0:
            print(f"[TTS] Piper error: {result.stderr.decode()}")
            return np.array([], dtype=np.float32)

        # Piper outputs raw 16-bit signed PCM at 22050 Hz
        audio_int16 = np.frombuffer(result.stdout, dtype=np.int16)
        audio_float = audio_int16.astype(np.float32) / 32768.0
        return audio_float

    def synthesize_to_file(self, text: str, output_path: str):
        """Synthesize text and save as WAV file."""
        audio = self.synthesize(text)
        if len(audio) == 0:
            return

        audio_int16 = (audio * 32767).astype(np.int16)
        with wave.open(output_path, "w") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_int16.tobytes())
