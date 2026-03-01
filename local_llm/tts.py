"""
Text-to-Speech module using Piper TTS (Python library).
Runs on CPU, leaving GPU free for LLM and Whisper.
Outputs raw PCM audio that can be played directly via PyAudio.
"""

import wave
import numpy as np
from pathlib import Path

# Default Piper voice model
DEFAULT_PIPER_MODEL = "en_US-libritts_r-medium"
DEFAULT_PIPER_DATA_DIR = Path(__file__).parent / "piper_models"
DEFAULT_SAMPLE_RATE = 22050


class TextToSpeech:
    def __init__(
        self,
        model: str = DEFAULT_PIPER_MODEL,
        data_dir: Path = DEFAULT_PIPER_DATA_DIR,
        sample_rate: int = DEFAULT_SAMPLE_RATE,
    ):
        self.model = model
        self.data_dir = data_dir
        self.sample_rate = sample_rate
        self.voice = None

    def load(self):
        from piper import PiperVoice

        model_path = self.data_dir / f"{self.model}.onnx"
        config_path = self.data_dir / f"{self.model}.onnx.json"

        if not model_path.exists():
            raise FileNotFoundError(
                f"Piper model not found at {model_path}. Run setup.sh to download models."
            )

        print(f"[TTS] Loading Piper voice model '{self.model}'...")
        self.voice = PiperVoice.load(str(model_path), config_path=str(config_path))
        self.sample_rate = self.voice.config.sample_rate
        print(f"[TTS] Piper ready (sample_rate={self.sample_rate})")

    def synthesize(self, text: str) -> np.ndarray:
        if self.voice is None:
            raise RuntimeError("Voice not loaded. Call load() first.")

        if not text or not text.strip():
            return np.array([], dtype=np.float32)

        # Collect raw PCM from all audio chunks (one per sentence)
        audio_chunks = []
        for chunk in self.voice.synthesize(text):
            audio_chunks.append(chunk.audio_int16_array)

        if not audio_chunks:
            return np.array([], dtype=np.float32)

        # chunk.audio is a numpy int16 array
        audio_int16 = np.concatenate(audio_chunks)
        audio_float = audio_int16.astype(np.float32) / 32768.0
        return audio_float

    def synthesize_to_file(self, text: str, output_path: str):
        """Synthesize text and save as WAV file."""
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
