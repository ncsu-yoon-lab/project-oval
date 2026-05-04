"""
Text-to-Speech using the Piper binary (subprocess).
Outputs raw 16-bit PCM audio for direct playback via PyAudio.

The Piper Python package does not work on aarch64 (Jetson Orin).
Using the pre-compiled binary installed by setup.sh instead.
"""

import subprocess
import wave
import numpy as np
from pathlib import Path

DEFAULT_PIPER_MODEL = "en_US-lessac-medium"
DEFAULT_PIPER_BINARY = Path(__file__).parent / "piper" / "piper"
DEFAULT_PIPER_DATA_DIR = Path(__file__).parent / "piper_models"
DEFAULT_SAMPLE_RATE = 22050


class TextToSpeech:
    def __init__(
        self,
        model: str = DEFAULT_PIPER_MODEL,
        data_dir: Path = DEFAULT_PIPER_DATA_DIR,
        piper_binary: Path = DEFAULT_PIPER_BINARY,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ):
        self.model = model
        self.data_dir = data_dir
        self.piper_binary = piper_binary
        self.sample_rate = sample_rate
        self._model_path = None
        self._config_path = None

    def load(self):
        if not self.piper_binary.exists():
            raise FileNotFoundError(
                f"Piper binary not found at {self.piper_binary}. Run setup.sh first."
            )

        self._model_path = self.data_dir / f"{self.model}.onnx"
        self._config_path = self.data_dir / f"{self.model}.onnx.json"

        if not self._model_path.exists():
            raise FileNotFoundError(
                f"Piper model not found at {self._model_path}. Run setup.sh to download models."
            )

        print(f"[TTS] Piper ready (model={self.model}, binary={self.piper_binary})")

    def synthesize(self, text: str) -> np.ndarray:
        if self._model_path is None:
            raise RuntimeError("Call load() before synthesize()")
        if not text or not text.strip():
            return np.array([], dtype=np.float32)

        cmd = [str(self.piper_binary), "--model", str(self._model_path), "--output-raw"]
        if self._config_path.exists():
            cmd += ["--config", str(self._config_path)]

        result = subprocess.run(
            cmd,
            input=text.encode(),
            capture_output=True,
            check=True,
        )

        audio_int16 = np.frombuffer(result.stdout, dtype=np.int16)
        return audio_int16.astype(np.float32) / 32768.0

    def synthesize_to_file(self, text: str, output_path: str):
        audio = self.synthesize(text)
        if len(audio) == 0:
            return

        audio_int16 = (audio * 32767).clip(-32768, 32767).astype(np.int16)
        with wave.open(output_path, "w") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(self.sample_rate)
            wf.writeframes(audio_int16.tobytes())
        print(f"[TTS] Saved to {output_path}")
