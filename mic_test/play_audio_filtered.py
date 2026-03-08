#!/usr/bin/env python3
"""
Filtered audio playback with noise reduction

Applies software filters to reduce noise from analog microphone
"""

import serial
import sounddevice as sd
import numpy as np
from scipy import signal
import sys
import time

SERIAL_PORT = '/dev/cu.usbserial-1120'
BAUD_RATE = 115200
SAMPLE_RATE = 8000
BUFFER_SIZE = 128

# Filter settings
ENABLE_LOWPASS = True      # Removes high-frequency hiss
ENABLE_HIGHPASS = True     # Removes low-frequency hum
ENABLE_DENOISE = True      # Simple noise gate

LOWPASS_FREQ = 3400        # Remove frequencies above 3.4kHz
HIGHPASS_FREQ = 300        # Remove frequencies below 300Hz
NOISE_GATE_THRESHOLD = 50  # Suppress signals below this level

class AudioFilter:
    def __init__(self):
        # Design filters
        if ENABLE_LOWPASS:
            # Low-pass filter (removes high-frequency noise)
            sos_lp = signal.butter(4, LOWPASS_FREQ, 'lowpass', fs=SAMPLE_RATE, output='sos')
            self.lowpass = signal.sosfilt_zi(sos_lp)
            self.sos_lp = sos_lp

        if ENABLE_HIGHPASS:
            # High-pass filter (removes low-frequency hum)
            sos_hp = signal.butter(2, HIGHPASS_FREQ, 'highpass', fs=SAMPLE_RATE, output='sos')
            self.highpass = signal.sosfilt_zi(sos_hp)
            self.sos_hp = sos_hp

    def filter_audio(self, audio):
        """Apply filters to audio buffer"""
        filtered = audio.astype(np.float32)

        # Low-pass filter
        if ENABLE_LOWPASS:
            filtered, self.lowpass = signal.sosfilt(self.sos_lp, filtered, zi=self.lowpass)

        # High-pass filter
        if ENABLE_HIGHPASS:
            filtered, self.highpass = signal.sosfilt(self.sos_hp, filtered, zi=self.highpass)

        # Simple noise gate
        if ENABLE_DENOISE:
            # Suppress very quiet signals (likely noise)
            mask = np.abs(filtered) < NOISE_GATE_THRESHOLD
            filtered[mask] *= 0.1  # Reduce but don't eliminate

        return filtered.astype(np.int16)

def main():
    print("\n🎤 Filtered Audio Streaming")
    print("="*60)
    print(f"Filters enabled:")
    print(f"  • Low-pass:  {ENABLE_LOWPASS} ({LOWPASS_FREQ}Hz)")
    print(f"  • High-pass: {ENABLE_HIGHPASS} ({HIGHPASS_FREQ}Hz)")
    print(f"  • Noise gate: {ENABLE_DENOISE} (threshold: {NOISE_GATE_THRESHOLD})")
    print("="*60 + "\n")

    audio_filter = AudioFilter()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print("Serial port opened")
        time.sleep(2)

        # Wait for ready
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if "AUDIO_STREAM_READY" in line:
                break

        print("\n✅ Streaming with filters enabled!")
        print("🎤 Speak into microphone - should be clearer now")
        print("Press Ctrl+C to stop\n")

        packet_count = 0

        with sd.OutputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16') as stream:
            while True:
                data = ser.read(BUFFER_SIZE * 2)

                if len(data) == BUFFER_SIZE * 2:
                    audio = np.frombuffer(data, dtype=np.int16)

                    # Apply filters
                    filtered_audio = audio_filter.filter_audio(audio)

                    # Play filtered audio
                    stream.write(filtered_audio)

                    # Progress indicator
                    packet_count += 1
                    if packet_count % 100 == 0:
                        print(f"Packets: {packet_count} | "
                              f"Input level: {np.abs(audio).mean():6.1f} | "
                              f"Output level: {np.abs(filtered_audio).mean():6.1f}")

    except KeyboardInterrupt:
        print("\n\n🛑 Stopped")
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'ser' in locals():
            ser.close()

if __name__ == "__main__":
    # Check dependencies
    try:
        import scipy
    except ImportError:
        print("ERROR: scipy not installed")
        print("\nInstall with:")
        print("  pip3 install scipy")
        sys.exit(1)

    try:
        import sounddevice
    except ImportError:
        print("ERROR: sounddevice not installed")
        print("  pip3 install sounddevice")
        sys.exit(1)

    main()
