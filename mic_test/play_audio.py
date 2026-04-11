#!/usr/bin/env python3
"""
INMP441 I2S Audio Playback — CASE Self-Balancing Robot
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Receives 16 kHz / 16-bit / mono audio from the ESP32 (audio_stream.ino)
over USB serial and plays it through your laptop speakers.

Requirements
────────────
    pip install pyserial sounddevice numpy

Usage
─────
    # Basic (auto-detects port, default 4× gain):
    python3 play_audio.py

    # Specify port:
    python3 play_audio.py --port /dev/cu.usbserial-1120

    # Increase volume (try 2–8):
    python3 play_audio.py --gain 6

    # List available serial ports:
    python3 play_audio.py --list-ports
"""

import sys
import time
import argparse

# ── Dependency guard ───────────────────────────────────────────
_missing = []
for _pkg, _imp in [("pyserial", "serial"), ("sounddevice", "sounddevice"), ("numpy", "numpy")]:
    try:
        __import__(_imp)
    except ImportError:
        _missing.append(_pkg)
if _missing:
    print(f"ERROR: Missing packages: {', '.join(_missing)}")
    print(f"  Install with:  pip install {' '.join(_missing)}")
    sys.exit(1)

import serial
import serial.tools.list_ports
import sounddevice as sd
import numpy as np

# ── Configuration — must match audio_stream.ino ───────────────
BAUD_RATE      = 921600     # High baud for 16 kHz stream
SAMPLE_RATE    = 16000      # Hz
BUFFER_SAMPLES = 256        # Samples per serial packet
PACKET_BYTES   = BUFFER_SAMPLES * 2  # 2 bytes per int16 sample
DEFAULT_GAIN   = 4.0        # INMP441 is quiet; 4× is a good starting point


# ── Serial port helpers ────────────────────────────────────────
def list_ports() -> None:
    ports = serial.tools.list_ports.comports()
    print("\nAvailable serial ports:")
    if not ports:
        print("  (none found — is the ESP32 plugged in?)")
    for p in ports:
        print(f"  {p.device:30s}  {p.description}")
    print()


def find_esp32_port() -> str | None:
    """Return the most likely ESP32 serial port, or None."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        return None

    # Known ESP32 USB-UART bridge identifiers
    KNOWN_KEYWORDS = ["cp210", "ch340", "ch341", "ftdi", "uart", "usb serial", "slab"]
    KNOWN_DEVICE   = ["usbserial", "ttyusb", "ttyacm", "slab"]

    for p in ports:
        desc = (p.description or "").lower()
        if any(k in desc for k in KNOWN_KEYWORDS):
            return p.device

    for p in ports:
        dev = p.device.lower()
        if any(k in dev for k in KNOWN_DEVICE):
            return p.device

    return ports[0].device  # fallback


# ── Main ──────────────────────────────────────────────────────
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stream INMP441 I2S audio from ESP32 to laptop speakers"
    )
    parser.add_argument("--port",       "-p", help="Serial port (auto-detected if omitted)")
    parser.add_argument("--gain",       "-g", type=float, default=DEFAULT_GAIN,
                        help=f"Volume gain multiplier (default: {DEFAULT_GAIN})")
    parser.add_argument("--list-ports", "-l", action="store_true",
                        help="Print available serial ports and exit")
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    # ── Port selection ─────────────────────────────────────
    port = args.port or find_esp32_port()
    if not port:
        print("ERROR: No serial port found.")
        list_ports()
        sys.exit(1)

    gain = args.gain

    # ── Banner ────────────────────────────────────────────
    print()
    print("━" * 54)
    print("  INMP441 I2S Audio Streaming — CASE Robot")
    print("━" * 54)
    print(f"  Port:         {port}")
    print(f"  Baud rate:    {BAUD_RATE:,}")
    print(f"  Sample rate:  {SAMPLE_RATE:,} Hz")
    print(f"  Bit depth:    16-bit signed")
    print(f"  Gain:         {gain}×  (adjust with --gain)")
    print("━" * 54)
    print()

    # ── Open serial ───────────────────────────────────────
    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=2)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open {port}: {e}")
        print()
        print("Troubleshooting:")
        print("  1. Close the Arduino Serial Monitor (it locks the port)")
        print("  2. Check port: python3 play_audio.py --list-ports")
        print("  3. Unplug / replug the ESP32")
        sys.exit(1)

    print(f"Serial port opened: {port}")
    print("Waiting for ESP32 to boot… (2 s)")
    time.sleep(2)
    ser.flushInput()

    # ── Handshake ─────────────────────────────────────────
    print("Waiting for handshake (AUDIO_STREAM_READY)…")
    deadline = time.time() + 10
    got_ready = False
    while time.time() < deadline:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            break
        if line:
            print(f"  ESP32: {line}")
        if "AUDIO_STREAM_READY" in line:
            got_ready = True
            break

    if not got_ready:
        print()
        print("ERROR: Timeout — did not receive AUDIO_STREAM_READY.")
        print("  • Is audio_stream.ino uploaded to the ESP32?")
        print("  • Is the baud rate set to 921600 in both files?")
        ser.close()
        sys.exit(1)

    print()
    print("Streaming active!")
    print("  Speak into the INMP441 microphone.")
    print("  Press Ctrl+C to stop.")
    print()

    # ── Audio stream loop ──────────────────────────────────
    total_packets = 0
    error_packets = 0
    STATUS_INTERVAL = SAMPLE_RATE // BUFFER_SAMPLES * 4  # every ~4 seconds

    try:
        with sd.OutputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype="int16",
            latency="low",
        ) as stream:

            while True:
                raw = ser.read(PACKET_BYTES)

                # Re-sync on incomplete packet
                if len(raw) != PACKET_BYTES:
                    error_packets += 1
                    ser.flushInput()
                    time.sleep(0.01)
                    continue

                audio = np.frombuffer(raw, dtype=np.int16).copy()

                # Apply gain (float path to avoid silent clipping)
                if gain != 1.0:
                    audio = np.clip(
                        audio.astype(np.float32) * gain,
                        -32768, 32767
                    ).astype(np.int16)

                stream.write(audio)
                total_packets += 1

                # ── Live level meter ───────────────────────
                if total_packets % STATUS_INTERVAL == 0:
                    rms   = int(np.sqrt(np.mean(audio.astype(np.float32) ** 2)))
                    peak  = int(np.abs(audio).max())
                    bars  = min(rms * 40 // 8000, 40)
                    print(
                        f"\r  [{('|' * bars):{40}s}]  "
                        f"RMS {rms:5d}  Peak {peak:6d}  "
                        f"Pkts {total_packets:6d}  Errs {error_packets}",
                        end="",
                        flush=True,
                    )

    except KeyboardInterrupt:
        print(f"\n\nStopped by user.")
        print(f"  Total packets : {total_packets}")
        print(f"  Error packets : {error_packets}")

    except sd.PortAudioError as e:
        print(f"\nAudio device error: {e}")
        print("Check that your laptop speakers are not muted / unplugged.")

    except serial.SerialException as e:
        print(f"\nSerial error: {e}")

    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback; traceback.print_exc()

    finally:
        if ser.is_open:
            ser.close()
            print("Serial port closed.")


if __name__ == "__main__":
    main()
