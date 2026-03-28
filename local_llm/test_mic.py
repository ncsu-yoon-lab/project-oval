#!/usr/bin/env python3
"""Quick diagnostic to check microphone input levels on this device."""

import pyaudio
import numpy as np
import sys

FORMAT = pyaudio.paInt16
CHANNELS = 1
DURATION_SEC = 5

pa = pyaudio.PyAudio()

# List all input devices
print("=== INPUT DEVICES ===")
default_idx = None
try:
    default_info = pa.get_default_input_device_info()
    default_idx = default_info['index']
except Exception:
    pass

for i in range(pa.get_device_count()):
    try:
        info = pa.get_device_info_by_index(i)
        if info['maxInputChannels'] > 0:
            marker = " <-- DEFAULT" if i == default_idx else ""
            print(f"  [{i}] {info['name']}  channels={info['maxInputChannels']}  rate={int(info['defaultSampleRate'])}Hz{marker}")
    except Exception:
        pass

# Pick device
device_idx = default_idx
rate = int(pa.get_device_info_by_index(device_idx)['defaultSampleRate'])

if len(sys.argv) > 1:
    device_idx = int(sys.argv[1])
    rate = int(pa.get_device_info_by_index(device_idx)['defaultSampleRate'])

print(f"\n=== TESTING DEVICE {device_idx} @ {rate}Hz for {DURATION_SEC}s ===")
print("Speak into the microphone now!\n")

chunk_size = int(rate * 0.032)  # ~32ms chunks
stream = pa.open(format=FORMAT, channels=CHANNELS, rate=rate,
                 input=True, input_device_index=device_idx,
                 frames_per_buffer=chunk_size)

num_chunks = int(DURATION_SEC * rate / chunk_size)
peak_overall = 0
rms_values = []

for i in range(num_chunks):
    raw = stream.read(chunk_size, exception_on_overflow=False)
    samples = np.frombuffer(raw, dtype=np.int16).astype(np.float32)
    
    peak = np.max(np.abs(samples))
    rms = np.sqrt(np.mean(samples ** 2))
    peak_overall = max(peak_overall, peak)
    rms_values.append(rms)
    
    # Print a simple VU meter every ~250ms
    if i % 8 == 0:
        bar_len = int(rms / 32768 * 100)
        bar = "#" * bar_len + "-" * (50 - min(bar_len, 50))
        print(f"  RMS: {rms:8.1f}  Peak: {peak:8.1f}  |{bar[:50]}|")

stream.stop_stream()
stream.close()
pa.terminate()

avg_rms = np.mean(rms_values)
print(f"\n=== RESULTS ===")
print(f"  Peak level:    {peak_overall:.0f} / 32768  ({peak_overall/32768*100:.1f}%)")
print(f"  Average RMS:   {avg_rms:.0f} / 32768  ({avg_rms/32768*100:.1f}%)")

if peak_overall < 100:
    print("\n  ⚠️  VERY LOW / SILENCE — Mic may not be connected or is muted!")
    print("     Try: arecord -l  (to list ALSA capture devices)")
    print("     Try: arecord -D hw:1,0 -f S16_LE -r 44100 -d 3 test.wav  (test a specific device)")
elif peak_overall < 1000:
    print("\n  ⚠️  LOW INPUT — Mic is picking up something but level is very low.")
    print("     Try increasing gain:  amixer -c 0 set Capture 100%")
    print("     Or try a different device:  python test_mic.py <device_index>")
elif peak_overall < 5000:
    print("\n  ℹ️  MODERATE INPUT — Detectable but could be louder.")
    print("     Speech detection should work. If not, try lowering SPEECH_THRESHOLD in vad.py")
else:
    print("\n  ✅  GOOD INPUT — Mic is working well!")
