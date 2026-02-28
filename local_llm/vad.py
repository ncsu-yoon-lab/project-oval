"""
Voice Activity Detection module using Silero VAD.
Detects when a user starts and stops speaking, then returns the captured audio.
Runs on CPU with minimal overhead.
"""

import numpy as np
import torch
import pyaudio
import time
import collections

# Audio config
SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16
# Silero VAD requires 512 samples at 16kHz (32ms chunks)
CHUNK_SIZE = 512

# VAD thresholds
SPEECH_THRESHOLD = 0.5
# How many consecutive silent chunks before we consider speech ended
SILENCE_CHUNKS_TO_END = 30  # ~960ms of silence
# Minimum speech duration to accept (in chunks)
MIN_SPEECH_CHUNKS = 10  # ~320ms
# Maximum recording duration in seconds
MAX_RECORDING_SECONDS = 30


class VoiceActivityDetector:
    """
    Listens to the microphone and returns audio when speech is detected.
    Uses Silero VAD to detect speech boundaries.
    """

    def __init__(self, sample_rate: int = SAMPLE_RATE, device_index: int = None):
        self.sample_rate = sample_rate
        self.device_index = device_index
        self.model = None
        self.pyaudio_instance = None

    def load(self):
        """Load Silero VAD model."""
        print("[VAD] Loading Silero VAD model...")
        self.model, utils = torch.hub.load(
            repo_or_dir="snakers4/silero-vad",
            model="silero_vad",
            force_reload=False,
            onnx=True,
        )
        print("[VAD] Silero VAD loaded.")

    def _reset_model(self):
        """Reset VAD model state between utterances."""
        self.model.reset_states()

    def _is_speech(self, audio_chunk: np.ndarray) -> bool:
        """Run VAD on a single chunk and return True if speech detected."""
        tensor = torch.from_numpy(audio_chunk).float()
        if tensor.max() > 1.0:
            tensor = tensor / 32768.0
        confidence = self.model(tensor, self.sample_rate).item()
        return confidence > SPEECH_THRESHOLD

    def listen_for_speech(self, pyaudio_instance: pyaudio.PyAudio) -> np.ndarray:
        """
        Block until the user speaks and finishes speaking.

        Returns:
            float32 numpy array of the captured speech audio at self.sample_rate.
            Returns empty array if no valid speech was captured.
        """
        self._reset_model()

        stream = pyaudio_instance.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=CHUNK_SIZE,
        )

        audio_buffer = []
        speech_started = False
        silence_count = 0
        speech_chunk_count = 0
        # Keep a small pre-buffer so we don't clip the start of speech
        pre_buffer = collections.deque(maxlen=5)
        max_chunks = int(MAX_RECORDING_SECONDS * self.sample_rate / CHUNK_SIZE)

        try:
            print("[VAD] Listening for speech...")
            for _ in range(max_chunks):
                raw = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16).astype(np.float32)

                is_speech = self._is_speech(chunk)

                if not speech_started:
                    pre_buffer.append(chunk)
                    if is_speech:
                        speech_started = True
                        silence_count = 0
                        speech_chunk_count = 1
                        # Include pre-buffer to capture onset
                        audio_buffer.extend(pre_buffer)
                        pre_buffer.clear()
                        print("[VAD] Speech detected!")
                else:
                    audio_buffer.append(chunk)
                    if is_speech:
                        speech_chunk_count += 1
                        silence_count = 0
                    else:
                        silence_count += 1
                        if silence_count >= SILENCE_CHUNKS_TO_END:
                            print("[VAD] End of speech detected.")
                            break
        finally:
            stream.stop_stream()
            stream.close()

        if speech_chunk_count < MIN_SPEECH_CHUNKS:
            print("[VAD] Speech too short, ignoring.")
            return np.array([], dtype=np.float32)

        # Concatenate and normalize
        audio = np.concatenate(audio_buffer)
        audio = audio / 32768.0
        return audio
