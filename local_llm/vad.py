"""
Voice Activity Detection module.
Uses Google's WebRTC VAD for robust speech detection.
"""

import numpy as np
import webrtcvad
import pyaudio
import collections

# Audio config
SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16

# WebRTC VAD requires 10, 20, or 30ms frames
FRAME_DURATION_MS = 30
FRAME_SIZE = int(SAMPLE_RATE * FRAME_DURATION_MS / 1000)  # 480 samples at 16kHz

# VAD aggressiveness: 0 (least aggressive) to 3 (most aggressive)
# Higher = more aggressive about filtering out non-speech
VAD_AGGRESSIVENESS = 2

# How many consecutive silent frames before speech ends (~800ms)
SILENCE_FRAMES_TO_END = int(800 / FRAME_DURATION_MS)  # ~27 frames

# Minimum speech duration to accept (~300ms)
MIN_SPEECH_FRAMES = int(300 / FRAME_DURATION_MS)  # ~10 frames

# Maximum recording duration
MAX_RECORDING_SECONDS = 30

# Ring buffer size for pre-speech padding (~300ms)
PRE_SPEECH_FRAMES = int(300 / FRAME_DURATION_MS)


class VoiceActivityDetector:
    """
    Listens to the microphone and returns audio when speech is detected.
    Uses Google's WebRTC VAD.
    """

    def __init__(self, sample_rate: int = SAMPLE_RATE, device_index: int = None):
        self.sample_rate = sample_rate
        self.device_index = device_index
        self.vad = None

    def load(self):
        """Initialize WebRTC VAD."""
        print(f"[VAD] Initializing WebRTC VAD (aggressiveness={VAD_AGGRESSIVENESS})...")
        self.vad = webrtcvad.Vad(VAD_AGGRESSIVENESS)
        print("[VAD] WebRTC VAD ready.")

    def calibrate(self, pyaudio_instance: pyaudio.PyAudio, duration: float = 1.0):
        """No calibration needed for WebRTC VAD, but keep the interface."""
        print("[VAD] WebRTC VAD does not require calibration. Skipping.")

    def listen_for_speech(self, pyaudio_instance: pyaudio.PyAudio) -> np.ndarray:
        """
        Block until the user speaks and finishes speaking.
        Returns float32 audio normalized to [-1, 1].
        """
        stream = pyaudio_instance.open(
            format=FORMAT, channels=CHANNELS, rate=48000,
            input=True, input_device_index=24,
            frames_per_buffer=FRAME_SIZE,
        )

        audio_buffer = []
        speech_started = False
        silence_count = 0
        speech_frame_count = 0
        pre_buffer = collections.deque(maxlen=PRE_SPEECH_FRAMES)
        max_frames = int(MAX_RECORDING_SECONDS * self.sample_rate / FRAME_SIZE)

        # Track voiced frames in a sliding window for more robust detection
        ring_buffer = collections.deque(maxlen=10)

        try:
            print("[VAD] Listening for speech...")
            for _ in range(max_frames):
                raw = stream.read(FRAME_SIZE, exception_on_overflow=False)

                # WebRTC VAD expects raw int16 bytes
                is_speech = self.vad.is_speech(raw, self.sample_rate)

                chunk = np.frombuffer(raw, dtype=np.int16).astype(np.float32)

                if not speech_started:
                    pre_buffer.append(chunk)
                    ring_buffer.append(is_speech)

                    # Require majority of recent frames to be speech before triggering
                    voiced_count = sum(ring_buffer)
                    if voiced_count >= 6:  # 6 out of 10 frames voiced
                        speech_started = True
                        silence_count = 0
                        speech_frame_count = voiced_count
                        audio_buffer.extend(pre_buffer)
                        pre_buffer.clear()
                        ring_buffer.clear()
                        print("[VAD] Speech detected!")
                else:
                    audio_buffer.append(chunk)
                    if is_speech:
                        speech_frame_count += 1
                        silence_count = 0
                    else:
                        silence_count += 1
                        if silence_count >= SILENCE_FRAMES_TO_END:
                            print("[VAD] End of speech detected.")
                            break
        finally:
            stream.stop_stream()
            stream.close()

        if speech_frame_count < MIN_SPEECH_FRAMES:
            print("[VAD] Speech too short, ignoring.")
            return np.array([], dtype=np.float32)

        audio = np.concatenate(audio_buffer)
        audio = audio / 32768.0

        # Normalize audio level so Whisper gets a strong signal
        peak = np.abs(audio).max()
        if peak > 0.001:
            audio = audio / peak * 0.9
            print(f"[VAD] Audio normalized (peak was {peak:.4f})")

        return audio
