"""
Voice Activity Detection module.
Uses energy-based pre-filtering + Silero VAD for robust speech detection.
Mimics the Gemini Live approach: low start sensitivity, high end sensitivity.
"""

import numpy as np
import torch
import pyaudio
import collections

# Audio config
SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16
# Silero VAD requires 512 samples at 16kHz (32ms chunks)
CHUNK_SIZE = 512

# Energy gate — chunks below this RMS are ignored before reaching Silero.
# This filters out quiet background noise so Silero only processes real audio.
# Calibrated on startup if possible.
ENERGY_THRESHOLD = 300  # RMS of int16 samples; typical silence is 50-200

# Silero VAD threshold — only chunks passing energy gate get checked
SPEECH_THRESHOLD = 0.5

# Speech start: require N consecutive speech chunks to trigger (reduces false starts)
START_CONFIRM_CHUNKS = 3  # ~96ms of confirmed speech before we commit

# Speech end: require N consecutive silent chunks (like Gemini's 800ms silence)
SILENCE_CHUNKS_TO_END = 25  # ~800ms of silence

# Minimum speech duration to accept
MIN_SPEECH_CHUNKS = 15  # ~480ms

# Maximum recording duration in seconds
MAX_RECORDING_SECONDS = 30


class VoiceActivityDetector:
    """
    Listens to the microphone and returns audio when speech is detected.
    Uses energy-based pre-filtering + Silero VAD for robustness.
    """

    def __init__(self, sample_rate: int = SAMPLE_RATE, device_index: int = None):
        self.sample_rate = sample_rate
        self.device_index = device_index
        self.model = None
        self.energy_threshold = ENERGY_THRESHOLD

    def load(self):
        """Load Silero VAD model."""
        print("[VAD] Loading Silero VAD model...")
        self.model, _ = torch.hub.load(
            repo_or_dir="snakers4/silero-vad",
            model="silero_vad",
            force_reload=False,
            onnx=True,
        )
        print("[VAD] Silero VAD loaded.")

    def calibrate(self, pyaudio_instance: pyaudio.PyAudio, duration: float = 1.0):
        """
        Calibrate the energy threshold based on ambient noise.
        Records a short sample of silence and sets the threshold above it.
        """
        print("[VAD] Calibrating noise floor (stay quiet)...")
        stream = pyaudio_instance.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=CHUNK_SIZE,
        )

        rms_values = []
        num_chunks = int(duration * self.sample_rate / CHUNK_SIZE)
        try:
            for _ in range(num_chunks):
                raw = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                rms = np.sqrt(np.mean(chunk ** 2))
                rms_values.append(rms)
        finally:
            stream.stop_stream()
            stream.close()

        avg_rms = np.mean(rms_values)
        # Set threshold at 3x the ambient noise floor
        self.energy_threshold = max(avg_rms * 3, 200)
        print(f"[VAD] Noise floor: {avg_rms:.0f}, threshold set to: {self.energy_threshold:.0f}")

    def _reset_model(self):
        """Reset VAD model state between utterances."""
        self.model.reset_states()

    def _chunk_energy(self, chunk: np.ndarray) -> float:
        """Calculate RMS energy of a chunk."""
        return np.sqrt(np.mean(chunk ** 2))

    def _is_speech(self, audio_chunk: np.ndarray) -> bool:
        """Run Silero VAD on a single chunk."""
        tensor = torch.from_numpy(audio_chunk).float()
        if tensor.max() > 1.0:
            tensor = tensor / 32768.0
        confidence = self.model(tensor, self.sample_rate).item()
        return confidence > SPEECH_THRESHOLD

    def listen_for_speech(self, pyaudio_instance: pyaudio.PyAudio) -> np.ndarray:
        """
        Block until the user speaks and finishes speaking.

        Uses a two-stage approach:
        1. Energy gate: ignore chunks below ambient noise threshold
        2. Silero VAD: confirm speech on chunks that pass the energy gate

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
        speech_confirming = False
        confirm_count = 0
        silence_count = 0
        speech_chunk_count = 0
        # Pre-buffer to capture audio before speech is confirmed
        pre_buffer = collections.deque(maxlen=10)
        max_chunks = int(MAX_RECORDING_SECONDS * self.sample_rate / CHUNK_SIZE)

        try:
            print("[VAD] Listening for speech...")
            for _ in range(max_chunks):
                raw = stream.read(CHUNK_SIZE, exception_on_overflow=False)
                chunk = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
                energy = self._chunk_energy(chunk)

                if not speech_started:
                    pre_buffer.append(chunk)

                    # Stage 1: Energy gate
                    if energy < self.energy_threshold:
                        # Too quiet — reset any pending confirmation
                        if speech_confirming:
                            confirm_count = 0
                            speech_confirming = False
                        continue

                    # Stage 2: Silero VAD confirmation
                    if self._is_speech(chunk):
                        if not speech_confirming:
                            speech_confirming = True
                            confirm_count = 1
                        else:
                            confirm_count += 1

                        # Require consecutive speech chunks to confirm start
                        if confirm_count >= START_CONFIRM_CHUNKS:
                            speech_started = True
                            speech_chunk_count = confirm_count
                            silence_count = 0
                            audio_buffer.extend(pre_buffer)
                            pre_buffer.clear()
                            print("[VAD] Speech detected!")
                    else:
                        confirm_count = 0
                        speech_confirming = False
                else:
                    # Already recording speech
                    audio_buffer.append(chunk)

                    if energy >= self.energy_threshold and self._is_speech(chunk):
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
