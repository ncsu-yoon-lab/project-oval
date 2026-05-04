"""
Wakeword detection using OpenWakeWord.
Listens continuously for the configured wakeword before activating the pipeline.
"""

from __future__ import annotations

from pathlib import Path
from typing import Optional
import numpy as np
import pyaudio

SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16
CHUNK_SIZE = 1280  # 80ms at 16kHz — required by OpenWakeWord

_TRAINED_MODEL = Path(__file__).parent / "models/hey_wolfwagen.onnx"

DEFAULT_WAKEWORD = "hey_wolfwagen" if _TRAINED_MODEL.exists() else "hey_jarvis"
DEFAULT_MODEL_PATH: Optional[str] = str(_TRAINED_MODEL) if _TRAINED_MODEL.exists() else None
DEFAULT_THRESHOLD = 0.7


class WakeWordDetector:
    """
    Continuously listens for a wakeword and returns when it is detected.
    Uses OpenWakeWord with a built-in or custom ONNX model.

    Built-in models: hey_jarvis, alexa, hey_mycroft, timer, weather
    Custom model:    pass model_path pointing to your .onnx file
    """

    def __init__(
        self,
        wakeword: str = DEFAULT_WAKEWORD,
        threshold: float = DEFAULT_THRESHOLD,
        device_index: Optional[int] = None,
        model_path: Optional[str] = DEFAULT_MODEL_PATH,
    ):
        self.wakeword = wakeword
        self.threshold = threshold
        self.device_index = device_index
        self.model_path = model_path
        self.model = None

    def load(self):
        """Download built-in models if needed, then load the wakeword model."""
        from openwakeword.model import Model
        import openwakeword

        print("[Wakeword] Downloading/verifying built-in models...")
        try:
            openwakeword.utils.download_models()  # no-op if already cached
        except AttributeError:
            pass  # older versions don't have this helper

        model_source = self.model_path if self.model_path else self.wakeword
        print(f"[Wakeword] Loading model '{model_source}'...")
        self.model = Model(
            wakeword_models=[model_source],
            inference_framework="onnx",
        )
        print("[Wakeword] Model ready.")

    def wait_for_wakeword(self, pyaudio_instance: pyaudio.PyAudio) -> bool:
        """
        Block until the wakeword is detected.
        Returns True on detection.
        """
        assert self.model is not None, "Call load() before wait_for_wakeword()"

        stream = pyaudio_instance.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=SAMPLE_RATE,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=CHUNK_SIZE,
        )

        print(f"[Wakeword] Waiting for '{self.wakeword}'...")
        try:
            while True:
                raw = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                audio_chunk = np.frombuffer(raw, dtype=np.int16)

                predictions: dict = self.model.predict(audio_chunk)  # type: ignore[union-attr]
                score = max(predictions.values(), default=0.0)

                if score >= self.threshold:
                    print(f"[Wakeword] Detected! (score={score:.3f})")
                    self._reset()
                    return True
        finally:
            stream.stop_stream()
            stream.close()

    def _reset(self):
        """Clear internal prediction buffer to prevent immediate re-triggering."""
        if self.model and hasattr(self.model, "prediction_buffer"):
            self.model.prediction_buffer.clear()  # type: ignore[union-attr]
