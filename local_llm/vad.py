"""
Voice Activity Detection module.
Uses Google's WebRTC VAD for robust speech detection.
"""

import numpy as np
import webrtcvad
import pyaudio
import collections

# Audio config — processing rate for VAD and Whisper
SAMPLE_RATE = 16000
CHANNELS = 1
FORMAT = pyaudio.paInt16

# Hardware rate: many USB mics on Jetson only support 48kHz natively.
# Audio is captured at HARDWARE_RATE then downsampled to SAMPLE_RATE.
HARDWARE_SAMPLE_RATE = 48000
DOWNSAMPLE_RATIO = HARDWARE_SAMPLE_RATE // SAMPLE_RATE  # 3

# WebRTC VAD requires 10, 20, or 30ms frames — sized for the hardware rate
FRAME_DURATION_MS = 30
FRAME_SIZE = int(HARDWARE_SAMPLE_RATE * FRAME_DURATION_MS / 1000)  # 1440 samples @ 48kHz

# Input device index for the Jetson USB microphone
MIC_DEVICE_INDEX = 24

# VAD aggressiveness: 0 (least aggressive) to 3 (most aggressive)
VAD_AGGRESSIVENESS = 2

# How many consecutive silent frames before speech ends (~800ms)
SILENCE_FRAMES_TO_END = int(800 / FRAME_DURATION_MS)  # ~27 frames

# Minimum speech duration to accept (~300ms)
MIN_SPEECH_FRAMES = int(300 / FRAME_DURATION_MS)  # ~10 frames

# Maximum recording duration
MAX_RECORDING_SECONDS = 30

# Ring buffer size for pre-speech padding (~300ms)
PRE_SPEECH_FRAMES = int(300 / FRAME_DURATION_MS)


def _downsample(chunk_48k: np.ndarray) -> np.ndarray:
    """Decimate 48kHz int16 chunk to 16kHz by taking every 3rd sample."""
    return chunk_48k[::DOWNSAMPLE_RATIO]


class VoiceActivityDetector:
    """
    Listens to the microphone and returns 16kHz audio when speech is detected.
    Captures at HARDWARE_SAMPLE_RATE (48kHz) and downsamples for VAD/Whisper.
    """

    def __init__(self, sample_rate: int = SAMPLE_RATE, device_index: int = MIC_DEVICE_INDEX):
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
        Returns float32 audio at 16kHz normalized to [-1, 1].
        """
        stream = pyaudio_instance.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=HARDWARE_SAMPLE_RATE,
            input=True,
            input_device_index=self.device_index,
            frames_per_buffer=FRAME_SIZE,
        )

        audio_buffer = []
        speech_started = False
        silence_count = 0
        speech_frame_count = 0
        pre_buffer = collections.deque(maxlen=PRE_SPEECH_FRAMES)
        max_frames = int(MAX_RECORDING_SECONDS * HARDWARE_SAMPLE_RATE / FRAME_SIZE)

        ring_buffer = collections.deque(maxlen=10)

        try:
            print("[VAD] Listening for speech...")
            for _ in range(max_frames):
                raw = stream.read(FRAME_SIZE, exception_on_overflow=False)

                # Downsample to 16kHz for WebRTC VAD
                chunk_48k = np.frombuffer(raw, dtype=np.int16)
                chunk_16k = _downsample(chunk_48k)
                raw_16k = chunk_16k.tobytes()

                is_speech = self.vad.is_speech(raw_16k, self.sample_rate)

                chunk_float = chunk_16k.astype(np.float32)

                if not speech_started:
                    pre_buffer.append(chunk_float)
                    ring_buffer.append(is_speech)

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
                    audio_buffer.append(chunk_float)
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

        peak = np.abs(audio).max()
        print(f"[VAD] Captured {len(audio)/self.sample_rate:.2f}s of speech (peak={peak:.4f})")

        return audio
