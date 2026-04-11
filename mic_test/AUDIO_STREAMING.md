# Audio Streaming: INMP441 → ESP32 → Laptop
**CASE Self-Balancing Robot — Audio Subsystem**

---

## Hardware

| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Microphone | INMP441 MEMS | I2S digital | Voice input |
| Amplifier + Speakers | PAM8403 + 2× 3W 8Ω | Analog (Class D) | Audio output |

---

## Complete Pinout

### INMP441 Microphone → ESP32

| INMP441 Pin | ESP32 GPIO | Description |
|-------------|------------|-------------|
| VDD | **3.3V** | Power — do **not** use 5V |
| GND | GND | Ground |
| L/R | GND | Channel select: GND = left channel |
| SCK | **GPIO 32** | I2S Bit Clock (BCLK) |
| WS | **GPIO 15** | I2S Word Select (LRCLK) |
| SD | **GPIO 4** | I2S Serial Data (output from mic) |

### PAM8403 Amplifier → ESP32

| PAM8403 Pin | Connect To | Description |
|-------------|------------|-------------|
| **5V** | 5V (VIN pin) | Power from USB / VIN rail |
| **GND** | GND | Power ground |
| **L** | GPIO 25 (DAC1) | Left audio input signal |
| **T** | GND | Audio common / signal ground |
| **R** | GPIO 25 (DAC1) | Right audio input — tie to L for mono (both speakers) |
| Left output pair (L+/L−) | Left speaker | 3W 8Ω speaker |
| Right output pair (R+/R−) | Right speaker | 3W 8Ω speaker |

> **Mono wiring:** Both **L** and **R** pins connect to GPIO 25 so both speakers
> reproduce the same mono audio. **T** is the signal ground — it must be wired to
> ESP32 GND, not left floating.

### Complete GPIO Allocation (Audio + Balance System)

| GPIO | Function | Component | Notes |
|------|----------|-----------|-------|
| 2 | MPU INT | MPU6050 | Interrupt — do not use |
| 4 | I2S SD | INMP441 | Mic data |
| **12** | BIN1 | TB6612FNG | Motor B direction |
| **13** | BIN2 | TB6612FNG | Motor B direction |
| **14** | PWMB | TB6612FNG | Motor B PWM |
| 15 | I2S WS | INMP441 | Mic word select |
| **16** | PWMA | TB6612FNG | Motor A PWM ← **moved from GPIO 25** |
| **17** | AIN1 | TB6612FNG | Motor A direction ← **moved from GPIO 26** |
| **19** | AIN2 | TB6612FNG | Motor A direction ← **moved from GPIO 27** |
| 21 | SDA | MPU6050 | I2C data |
| 22 | SCL | MPU6050 | I2C clock |
| **25** | DAC1 | PAM8403 | Audio output (analog) |
| 32 | I2S SCK | INMP441 | Mic bit clock |
| **33** | STBY | TB6612FNG | Motor driver standby |
| 3.3V | VDD | INMP441, MPU6050 | 3.3V rail |
| 5V (VIN) | VCC | PAM8403, TB6612FNG | 5V/motor rail |

> **IMPORTANT:** When integrating the audio subsystem with `self_balance.ino`,
> Motor A must be rewired to GPIO 16 (PWMA), GPIO 17 (AIN1), GPIO 19 (AIN2)
> to free GPIO 25 for the DAC audio output.

---

## Schematic (ASCII)

```
3.3V ─────────────┬──────────────────────────────────────┐
                  │                                      │
            INMP441                                 MPU6050
            ┌────────┐                              ┌───────┐
3.3V ──────▶│ VDD    │                    GPIO 21 ──│ SDA   │
GND ───────▶│ GND    │                    GPIO 22 ──│ SCL   │
GND ───────▶│ L/R    │                    GPIO 2  ──│ INT   │
GPIO 32 ───▶│ SCK    │                    3.3V ─────│ VCC   │
GPIO 15 ───▶│ WS     │                    GND ──────│ GND   │
GPIO 4  ◀──│ SD     │                              └───────┘
            └────────┘

                         PAM8403 module pin labels:
                         ┌──────────────────────────┐
5V (VIN) ─────────────▶│ 5V                        │
GND ──────────────┬───▶│ GND                       │
                  │     │                            │
GPIO 25 ─────────┬┼───▶│ L   (left audio in)       │──▶ L+ ──▶ Left Speaker
                 ││     │ T   (audio signal GND)    │──▶ L− ──▶ Left Speaker
                 │└────▶│     ← GND                 │
                 └─────▶│ R   (right audio in)      │──▶ R+ ──▶ Right Speaker
                         │                            │──▶ R− ──▶ Right Speaker
                         └──────────────────────────┘

TB6612FNG Motor Driver
GPIO 16 ─── PWMA      MOTORA+ ─── Left Motor
GPIO 17 ─── AIN1      MOTORA− ─── Left Motor
GPIO 19 ─── AIN2
GPIO 14 ─── PWMB      MOTORB+ ─── Right Motor
GPIO 12 ─── BIN1      MOTORB− ─── Right Motor
GPIO 13 ─── BIN2
GPIO 33 ─── STBY
```

---

## Sketch Selection Guide

| Sketch | Purpose | Laptop needed? |
|--------|---------|----------------|
| `inmp441_test.ino` | Verify INMP441 is wired and working | Yes (Serial Monitor) |
| `audio_stream.ino` | Stream mic audio to laptop speakers | Yes (Python script) |
| `audio_loopback.ino` | Direct mic → speaker test | No (standalone) |

---

## Quick Start: Streaming to Laptop

### Step 1 — Install Python dependencies
```bash
pip install pyserial sounddevice numpy
```

### Step 2 — Upload `audio_stream.ino`
Open in Arduino IDE → select your ESP32 board → Upload.

### Step 3 — Close Arduino Serial Monitor
The Serial Monitor locks the port. Python cannot connect while it is open.

### Step 4 — Run the Python script
```bash
cd /Users/samir/Projects/TARS-CASE/mic_test
python3 play_audio.py
```

### Step 5 — Speak into the INMP441
You should hear your voice through the laptop speakers within ~250 ms.

---

## Quick Start: Standalone Loopback (No Laptop)

1. Wire INMP441 and PAM8403 as shown in the pinout above.
2. Upload `audio_loopback.ino`.
3. Speak into the INMP441 → hear your voice through the PAM8403 speakers.
4. Adjust the PAM8403 potentiometer for comfortable volume.

---

## Troubleshooting

### "No serial port found"
```bash
python3 play_audio.py --list-ports
```
Look for a port named `usbserial-XXXX` (macOS) or `ttyUSB0` (Linux).
Specify it manually:
```bash
python3 play_audio.py --port /dev/cu.usbserial-1120
```

### "Timeout — did not receive AUDIO_STREAM_READY"
- Make sure `audio_stream.ino` (not `inmp441_test.ino`) is uploaded.
- Baud rate in the sketch (`BAUD_RATE 921600`) must match Python (`BAUD_RATE = 921600`).
- Try pressing the ESP32 Reset button after closing the Serial Monitor.

### Audio is very quiet
The INMP441 is a precision microphone with conservative output gain.
Increase software gain:
```bash
python3 play_audio.py --gain 8
```

### Audio is choppy or has gaps
Reduce gain (excessive gain → clipping → distortion).
If still choppy, the USB serial buffer may be too slow; try a powered hub.

### No sound in loopback (audio_loopback.ino)
- Confirm GPIO 25 is wired to PAM8403 `IN+` (both channels).
- Turn the PAM8403 volume potentiometer clockwise.
- Verify PAM8403 is powered from the 5V (VIN) pin, not 3.3V.
- Check Serial Monitor (115200 baud) — you should see peak levels change when speaking.

### I2S init fails (error code printed on Serial Monitor)
- Check VDD is 3.3V (not 5V) on INMP441.
- Recheck SCK → GPIO 32, WS → GPIO 15, SD → GPIO 4.
- Try a hard reset: unplug USB, rewire, replug.

---

## Audio Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Sample rate | 16 000 Hz | Voice quality (proposal target) |
| Bit depth | 24-bit (INMP441) → 16-bit (serial / DAC) | Truncated for transmission |
| Channel | Mono (left) | L/R pin tied to GND |
| Baud rate | 921 600 | Required for 16 kHz over USB serial |
| DAC resolution | 8-bit (ESP32 built-in) | Sufficient for voice; upgrade to I2S DAC for music |
| Latency (stream) | ~250 ms | Serial + sounddevice buffering |
| Latency (loopback) | ~130 ms | DMA buffer only |

---

## Technical Notes

### Why 921 600 baud?
At 16 kHz × 16-bit × 1 channel = 32 000 bytes/sec of audio data.
At 115 200 baud (10 bits/byte) = 11 520 bytes/sec max — **not enough**.
At 921 600 baud = 92 160 bytes/sec — 35% utilisation, plenty of headroom.

### INMP441 I2S format
The INMP441 outputs **24-bit audio left-aligned** in a 32-bit I2S slot.
- Bits 31–8: audio data (signed)
- Bits 7–0: always zero

`audio_stream.ino` shifts right by 16 to produce a signed 16-bit value.
`audio_loopback.ino` shifts right by 24 and offsets by +128 for the ESP32 DAC (8-bit unsigned).

### PAM8403 input
The PAM8403 accepts a differential analog input.
With `IN+` driven by ESP32 DAC (GPIO 25, 0–3.3 V) and `IN−` tied to GND,
the module operates in single-ended mode.
Most PAM8403 modules have on-board DC-blocking capacitors, so the ESP32 DAC
bias (~1.65 V at silence) does not affect the amplifier.
