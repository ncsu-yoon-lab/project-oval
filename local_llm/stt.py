"""
Speech-to-Text module using faster-whisper (CTranslate2 backend).
Runs on Jetson Orin with CUDA acceleration.
Falls back to CPU if CUDA is unavailable.
"""

import numpy as np
from faster_whisper import WhisperModel

# Default model config
DEFAULT_MODEL_SIZE = "base.en"
DEFAULT_DEVICE = "cuda"
DEFAULT_COMPUTE_TYPE = "float16"


class SpeechToText:
    def __init__(
        self,
        model_size: str = DEFAULT_MODEL_SIZE,
        device: str = DEFAULT_DEVICE,
        compute_type: str = DEFAULT_COMPUTE_TYPE,
    ):
        self.model_size = model_size
        self.device = device
        self.compute_type = compute_type
        self.model = None

    def load(self):
        print(
            f"[STT] Loading faster-whisper model '{self.model_size}' on {self.device}..."
        )
        try:
            self.model = WhisperModel(
                self.model_size,
                device=self.device,
                compute_type=self.compute_type,
            )
        except Exception:
            print("[STT] CUDA unavailable, falling back to CPU with int8...")
            self.device = "cpu"
            self.compute_type = "int8"
            self.model = WhisperModel(
                self.model_size,
                device=self.device,
                compute_type=self.compute_type,
            )
        print("[STT] Model loaded.")

    def transcribe(self, audio: np.ndarray, sample_rate: int = 16000) -> str:
        if self.model is None:
            raise RuntimeError("Model not loaded. Call load() first.")

        # faster-whisper expects float32 numpy array
        if audio.dtype != np.float32:
            audio = audio.astype(np.float32)

        # Normalize int16 range to float if needed
        if audio.max() > 1.0 or audio.min() < -1.0:
            audio = audio / 32768.0

        segments, info = self.model.transcribe(
            audio,
            beam_size=5,
            language="en",
            vad_filter=False,  # WebRTC VAD already handled segmentation
        )

        text = " ".join(segment.text.strip() for segment in segments)
        return text.strip()
