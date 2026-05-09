# TARS-CASE Architecture: Migration from ESP32-Only to Hybrid

## Executive Summary

The ESP32-WROOM-32 cannot sustain a real-time AI voice pipeline. With only
**167 KB free heap** after WiFi+TLS, the system is limited to **1.5 s audio
recordings**, HTTP POST round-trips (no streaming), and a Colab bridge proxy
(`personaplex_server.py`) to transcode between raw PCM and Opus. These are
fundamental RAM and protocol constraints that cannot be optimized away.

**Recommendation: Option A (Raspberry Pi 3 + ESP32 hybrid).**

| Factor | Why Option A wins |
|--------|-------------------|
| Balance fidelity | ESP32 interrupt-driven DMP loop has <10 us jitter; Linux kernel gives 2-15 ms |
| Code reuse | ~80 % of `self_balance.ino` runs unchanged on ESP32 |
| Voice pipeline | Pi connects to Moshi WebSocket directly -- eliminates `personaplex_server.py` entirely |
| Fault isolation | Pi crash does not topple the robot; ESP32 keeps balancing |
| Migration effort | ~1 week; Option B requires full PID rewrite + software PWM tuning |

---

## 1. Architecture Diagrams

### Option A: Raspberry Pi 3 + ESP32 Hybrid

```
 ┌──────────────────────────────────────────────────────────────┐
 │                     Raspberry Pi 3B+                         │
 │                                                              │
 │  ┌─────────────┐  ┌──────────────┐  ┌───────────────────┐   │
 │  │ Moshi WS    │  │ Wake-word /  │  │ UART command      │   │
 │  │ client      │──│ NLP pipeline │──│ dispatcher        │   │
 │  │ (Python)    │  │ (Python)     │  │ (pyserial)        │   │
 │  └─────────────┘  └──────────────┘  └────────┬──────────┘   │
 │        │                                      │ TX/RX        │
 │  ┌─────┴──────┐                               │              │
 │  │ I2S audio  │                               │              │
 │  │ INMP441 in │                               │              │
 │  │ MAX98357 out│                              │              │
 │  └────────────┘                               │              │
 └───────────────────────────────────────────────┼──────────────┘
                                                 │ UART 115200
 ┌───────────────────────────────────────────────┼──────────────┐
 │                      ESP32-WROOM-32           │              │
 │                                               │              │
 │  ┌──────────────┐  ┌────────────┐  ┌─────────┴──────────┐   │
 │  │ MPU6050 DMP  │──│ PID loop   │──│ UART command       │   │
 │  │ (I2C, INT)   │  │ (<10 us)   │  │ parser             │   │
 │  └──────────────┘  └─────┬──────┘  └────────────────────┘   │
 │                          │                                   │
 │                   ┌──────┴──────┐                            │
 │                   │ TB6612FNG   │                            │
 │                   │ Motor A / B │                            │
 │                   └─────────────┘                            │
 └──────────────────────────────────────────────────────────────┘
```

### Option B: Raspberry Pi 3 Only

```
 ┌──────────────────────────────────────────────────────────────┐
 │                     Raspberry Pi 3B+                         │
 │                                                              │
 │  ┌─────────────┐  ┌──────────────┐  ┌───────────────────┐   │
 │  │ Moshi WS    │  │ Wake-word /  │  │ PID loop          │   │
 │  │ client      │──│ NLP pipeline │  │ (pigpio SW PWM)   │   │
 │  │ (Python)    │  │ (Python)     │  │ 2-15 ms jitter    │   │
 │  └─────────────┘  └──────────────┘  └────────┬──────────┘   │
 │        │                                      │              │
 │  ┌─────┴──────┐  ┌──────────────┐      ┌─────┴──────────┐   │
 │  │ I2S audio  │  │ MPU6050      │      │ TB6612FNG      │   │
 │  │ INMP441 in │  │ (I2C, polled │      │ Motor A / B    │   │
 │  │ MAX98357 out│  │  or threaded)│      │ (SW PWM)       │   │
 │  └────────────┘  └──────────────┘      └────────────────┘   │
 └──────────────────────────────────────────────────────────────┘
```

---

## 2. Pin Configuration

### Option A: Pin Assignments

#### Raspberry Pi 3 GPIO

| Function | BCM Pin | Physical Pin | Direction | Notes |
|----------|---------|-------------|-----------|-------|
| UART TX | GPIO 14 | 8 | Output | -> ESP32 RX |
| UART RX | GPIO 15 | 10 | Input | <- ESP32 TX |
| I2S BCLK (mic) | GPIO 18 | 12 | Output | INMP441 SCK |
| I2S LRCLK (mic) | GPIO 19 | 35 | Output | INMP441 WS |
| I2S DIN (mic) | GPIO 20 | 38 | Input | INMP441 SD |
| I2S BCLK (spk) | GPIO 18 | 12 | Output | shared with mic BCLK |
| I2S DOUT (spk) | GPIO 21 | 40 | Output | MAX98357A DIN |
| I2S LRCLK (spk) | GPIO 19 | 35 | Output | shared with mic LRCLK |

> **I2S note:** The Pi has a single I2S peripheral. Mic and speaker share
> BCLK (GPIO 18) and LRCLK (GPIO 19). The mic reads on GPIO 20 (data in)
> and the speaker writes on GPIO 21 (data out). This works because I2S is
> full-duplex on the Pi's PCM/I2S interface.

#### ESP32-WROOM-32 GPIO (Option A -- balance only)

| Function | GPIO | Direction | Notes |
|----------|------|-----------|-------|
| I2C SDA | 21 | Bidir | MPU6050 |
| I2C SCL | 22 | Output | MPU6050 |
| MPU INT | 2 | Input | DMP interrupt |
| Motor A PWM | 25 | Output | TB6612FNG PWMA |
| Motor A IN1 | 26 | Output | TB6612FNG AIN1 |
| Motor A IN2 | 27 | Output | TB6612FNG AIN2 |
| Motor B PWM | 14 | Output | TB6612FNG PWMB |
| Motor B IN1 | 12 | Output | TB6612FNG BIN1 |
| Motor B IN2 | 13 | Output | TB6612FNG BIN2 |
| Standby | 33 | Output | TB6612FNG STBY (HIGH = enabled) |
| UART RX | 3 (RX0) | Input | <- Pi TX (GPIO 14) |
| UART TX | 1 (TX0) | Output | -> Pi RX (GPIO 15) |

> **Freed pins:** GPIO 4, 15, 32 (I2S mic), GPIO 18, 19, 23 (I2S speaker),
> and all WiFi resources. These are no longer needed on the ESP32.

### Option B: Raspberry Pi 3 GPIO (all functions)

| Function | BCM Pin | Physical Pin | Direction | Notes |
|----------|---------|-------------|-----------|-------|
| I2C SDA | GPIO 2 | 3 | Bidir | MPU6050 |
| I2C SCL | GPIO 3 | 5 | Output | MPU6050 |
| MPU INT | GPIO 4 | 7 | Input | DMP interrupt (pigpio callback) |
| Motor A PWM | GPIO 12 | 32 | Output | SW PWM via pigpio |
| Motor A IN1 | GPIO 5 | 29 | Output | TB6612FNG AIN1 |
| Motor A IN2 | GPIO 6 | 31 | Output | TB6612FNG AIN2 |
| Motor B PWM | GPIO 13 | 33 | Output | SW PWM via pigpio |
| Motor B IN1 | GPIO 16 | 36 | Output | TB6612FNG BIN1 |
| Motor B IN2 | GPIO 26 | 37 | Output | TB6612FNG BIN2 |
| Standby | GPIO 17 | 11 | Output | TB6612FNG STBY |
| I2S BCLK | GPIO 18 | 12 | Output | **Conflict:** also HW PWM ch0 |
| I2S LRCLK | GPIO 19 | 35 | Output | **Conflict:** also HW PWM ch1 |
| I2S DIN (mic) | GPIO 20 | 38 | Input | INMP441 SD |
| I2S DOUT (spk) | GPIO 21 | 40 | Output | MAX98357A DIN |

> **Pin conflict:** GPIO 18 and GPIO 19 are the Pi's only two hardware PWM
> channels. Using them for I2S forces motor PWM through pigpio's software
> PWM, which runs in userspace and adds 2-15 ms jitter to the PID loop.

---

## 3. Circuit Connection Diagrams

### Option A: UART Bridge Wiring

```
  Raspberry Pi 3                ESP32-WROOM-32
  ┌──────────────┐              ┌──────────────┐
  │              │              │              │
  │  GPIO 14 TX ├──────────────┤ GPIO 3  RX0  │
  │              │              │              │
  │  GPIO 15 RX ├──────────────┤ GPIO 1  TX0  │
  │              │              │              │
  │  GND        ├──────────────┤ GND          │
  │              │              │              │
  └──────────────┘              └──────────────┘

  UART config: 115200 baud, 8N1
  Logic levels: Pi = 3.3 V, ESP32 = 3.3 V (no level shifter needed)
```

### Option A: Pi Audio Wiring

```
  INMP441 Mic          Raspberry Pi 3           MAX98357A Amp
  ┌──────────┐         ┌──────────────┐         ┌──────────┐
  │ VDD      ├────3.3V─┤ 3.3V (pin 1) │         │ VIN      ├── 5V
  │ GND      ├────GND──┤ GND  (pin 6) ├────GND──┤ GND      │
  │ SCK      ├─────────┤ GPIO 18 BCLK │─────────┤ BCLK     │
  │ WS       ├─────────┤ GPIO 19 LRCLK│─────────┤ LRC      │
  │ SD       ├─────────┤ GPIO 20 DIN  │         │ DIN      ├── GPIO 21
  │ L/R      ├────GND  │              │         │ GAIN     ├── GND (9dB)
  └──────────┘         └──────────────┘         │ SD       ├── VIN
                                                │ +        ├── Speaker+
                                                │ -        ├── Speaker-
                                                └──────────┘
```

### Option A: ESP32 Balance Wiring (unchanged from current)

```
  MPU6050              ESP32                    TB6612FNG
  ┌──────────┐         ┌──────────────┐         ┌──────────────┐
  │ VCC      ├──3.3V───┤ 3.3V         │         │ VCC    ├─3.3V│
  │ GND      ├──GND────┤ GND          │         │ GND    ├─GND │
  │ SDA      ├─────────┤ GPIO 21      │         │ VM     ├─7.4V│
  │ SCL      ├─────────┤ GPIO 22      │         │ STBY   ├─G33 │
  │ INT      ├─────────┤ GPIO 2       │         │        │     │
  │ AD0      ├──GND    │              │         │ PWMA   ├─G25 │
  └──────────┘         │              │         │ AIN1   ├─G26 │
                       │              │         │ AIN2   ├─G27 │
                       │              │         │ PWMB   ├─G14 │
                       │              │         │ BIN1   ├─G12 │
                       │              │         │ BIN2   ├─G13 │
                       └──────────────┘         │        │     │
                                                │ AO1/2  ├─MotA│
                                                │ BO1/2  ├─MotB│
                                                └────────┴─────┘
```

### Power Distribution (Option A)

```
  ┌──────────────────┐
  │ 7.4V LiPo 2S     │
  │ (2000 mAh)       │
  └──┬──────┬────┬───┘
     │      │    │
     │      │    └──→ TB6612FNG VM (motor power)
     │      │
     │      └──→ 5V Buck Converter
     │           ├──→ Raspberry Pi 3 (5V micro-USB or GPIO pin 2/4)
     │           └──→ MAX98357A VIN
     │
     └──→ GND (common ground bus for ALL boards)
```

---

## 4. Performance Comparison

| Metric | Option A (Hybrid) | Option B (Pi Only) |
|--------|-------------------|--------------------|
| PID loop jitter | <10 us (HW interrupt) | 2-15 ms (Linux userspace) |
| PID loop rate | 100 Hz (DMP-driven) | 50-100 Hz (best effort) |
| Balance fault isolation | ESP32 keeps balancing if Pi crashes | Robot falls if any process stalls |
| Audio→Moshi latency | ~80 ms (Pi WebSocket direct) | ~80 ms (same) |
| Available RAM for AI | 1 GB (Pi) | 1 GB (Pi) |
| WiFi/TLS overhead | 0 on ESP32 (Pi handles all networking) | 0 on ESP32 (no ESP32) |
| Codec support | Pi runs Opus natively via sphn | Same |
| Real-time audio | WebSocket streaming | Same |
| Max recording length | Limited by Pi RAM (~minutes) | Same |
| Motor PWM type | Hardware (ESP32 LEDC) | Software (pigpio, userspace) |
| I2C bus speed | 400 kHz (ESP32 HW I2C) | 400 kHz (Pi HW I2C) |
| DMP offload | MPU6050 DMP on ESP32 | MPU6050 DMP on Pi (polled) |

---

## 5. Complexity Analysis

| Factor | Option A (Hybrid) | Option B (Pi Only) |
|--------|-------------------|--------------------|
| Migration time | ~1 week | ~2-3 weeks |
| ESP32 code reuse | ~80 % (strip WiFi/audio, add UART) | 0 % (all new Python) |
| New code required | UART protocol + Pi voice client | PID in Python, SW PWM driver, voice client |
| PID tuning | Kp/Ki/Kd carry over from `self_balance.ino` | Must re-tune for different loop timing |
| Testing risk | Low (balance loop proven) | High (new PID + jitter) |
| Ongoing maintenance | Two codebases (Arduino + Python) | One codebase (Python) |
| Debug complexity | UART logging + Serial Monitor | Single systemd journal |

---

## 6. Detailed Pros and Cons

### Option A: Raspberry Pi 3 + ESP32 Hybrid

**Pros:**
- Proven PID constants (Kp=15, Ki=140, Kd=0.9) transfer directly
- ESP32 DMP interrupt loop cannot be replicated in Linux userspace
- If the Pi kernel panics or Python crashes, the robot stays upright
- Pi connects to Moshi WebSocket natively -- eliminates `personaplex_server.py`
- 1 GB RAM for NLP, wake-word, and audio buffering
- ESP32 WiFi/BLE radio disabled -- saves power and reduces RF interference
- Clean separation: ESP32 = real-time body, Pi = AI brain

**Cons:**
- Two boards to power, mount, and debug
- UART protocol adds a communication layer to design and test
- Slightly higher power draw (~1.8 A for Pi + ESP32 vs ~1.2 A for Pi alone)
- More physical space needed in chassis

### Option B: Raspberry Pi 3 Only

**Pros:**
- Single board -- simpler wiring, fewer failure points
- One codebase (all Python)
- Lower power consumption (~1.2 A vs ~1.8 A)
- Easier remote debugging (SSH into one board)

**Cons:**
- **GPIO 18/19 conflict:** I2S clock pins overlap hardware PWM channels
- Software PWM jitter (2-15 ms) may cause balance oscillation
- PID constants must be re-tuned from scratch for different loop timing
- No fault isolation -- a stalled process topples the robot
- Must poll MPU6050 DMP from userspace (no true hardware interrupts)
- Python GIL can block PID loop during audio processing
- Full PID rewrite required (0 % code reuse from `self_balance.ino`)

---

## 7. Bill of Materials Comparison

### Option A: Hybrid

| Component | Qty | Est. Cost | Notes |
|-----------|-----|-----------|-------|
| Raspberry Pi 3B+ | 1 | $35 | AI brain |
| ESP32-WROOM-32 (existing) | 1 | $0 | Already owned |
| INMP441 I2S Microphone | 1 | $3 | Moves to Pi |
| MAX98357A I2S Amplifier | 1 | $4 | Moves to Pi |
| Speaker (4 ohm, 3W) | 1 | $3 | Existing |
| MPU6050 (existing) | 1 | $0 | Stays on ESP32 |
| TB6612FNG (existing) | 1 | $0 | Stays on ESP32 |
| 16 GB microSD card | 1 | $8 | Pi OS |
| Dupont jumper wires (UART) | 3 | $0 | TX, RX, GND |
| 5V 3A Buck Converter | 1 | $5 | Shared Pi + amp |
| **Total** | | **~$58** | |

### Option B: Pi Only

| Component | Qty | Est. Cost | Notes |
|-----------|-----|-----------|-------|
| Raspberry Pi 3B+ | 1 | $35 | Everything |
| INMP441 I2S Microphone | 1 | $3 | |
| MAX98357A I2S Amplifier | 1 | $4 | |
| Speaker (4 ohm, 3W) | 1 | $3 | Existing |
| MPU6050 (existing) | 1 | $0 | Moves to Pi I2C |
| TB6612FNG (existing) | 1 | $0 | Moves to Pi GPIO |
| 16 GB microSD card | 1 | $8 | |
| 5V 3A Buck Converter | 1 | $5 | |
| **Total** | | **~$58** | |

> BOM cost is identical. The difference is migration time and balance quality.

---

## 8. Software Stack Comparison

### Option A: Hybrid

| Layer | Pi (Brain) | ESP32 (Body) |
|-------|-----------|--------------|
| OS | Raspberry Pi OS Lite (Bookworm) | Bare metal (Arduino) |
| Language | Python 3.11 | C++ (Arduino framework) |
| Voice AI | `moshi-client` (WebSocket, Opus via `sphn`) | -- |
| Wake word | `openwakeword` or energy-based VAD | -- |
| Audio I/O | `sounddevice` + ALSA I2S driver | -- |
| Motor control | -- | `PID_v1.h`, `MPU6050_6Axis_MotionApps20.h` |
| Communication | `pyserial` (UART TX/RX) | `Serial` (UART RX/TX) |
| Process mgmt | `systemd` services | `loop()` + ISR |
| Logging | `journalctl` | Serial Monitor (via Pi USB or UART) |

### Option B: Pi Only

| Layer | Pi |
|-------|-----|
| OS | Raspberry Pi OS Lite (Bookworm) |
| Language | Python 3.11 |
| Voice AI | `moshi-client` (WebSocket, Opus via `sphn`) |
| Wake word | `openwakeword` or energy-based VAD |
| Audio I/O | `sounddevice` + ALSA I2S driver |
| IMU driver | `smbus2` (I2C to MPU6050) + DMP library |
| Motor control | `pigpio` (software PWM), custom PID in Python |
| Process mgmt | `systemd` services |
| Logging | `journalctl` |

---

## 9. Migration Guide: ESP32-Only to Option A (Hybrid)

### Phase 1: Prepare the ESP32 (Day 1-2)

1. **Fork `self_balance.ino` to `self_balance_uart.ino`**
   - Remove all WiFi, I2S, and audio includes
   - Remove `audio_input.h`, `audio_output.h`, `wake_word.h`, `command_classifier.h`, `colab_client.h`
   - Remove FreeRTOS dual-core task pinning (single core is sufficient)

2. **Add UART command parser**
   - Use `Serial` (UART0, GPIO 1/3) at 115200 baud
   - Parse incoming commands from Pi (see UART Protocol, Section 11)
   - Send telemetry (angle, PID output) back to Pi

3. **Verify balance still works standalone**
   - Upload, confirm PID constants (Kp=15, Ki=140, Kd=0.9) unchanged
   - Confirm motor pins: PWMA=25, AIN1=26, AIN2=27, PWMB=14, BIN1=12, BIN2=13, STBY=33
   - Confirm IMU pins: SDA=21, SCL=22, INT=2

### Phase 2: Set Up Raspberry Pi (Day 2-3)

4. **Install Raspberry Pi OS Lite (Bookworm, 64-bit)**
   - Enable I2S overlay in `/boot/config.txt`:
     ```
     dtoverlay=i2s-mmap
     dtoverlay=googlevoicehat-soundcard
     ```
     Or use a generic I2S overlay for INMP441 + MAX98357A.

5. **Enable UART on Pi**
   - In `/boot/config.txt`:
     ```
     enable_uart=1
     dtoverlay=disable-bt
     ```
   - This maps `/dev/ttyAMA0` to GPIO 14 (TX) / GPIO 15 (RX)
   - Disable serial console: remove `console=serial0,115200` from `/boot/cmdline.txt`

6. **Install Python dependencies**
   ```bash
   sudo apt update && sudo apt install -y python3-pip python3-venv libasound2-dev
   python3 -m venv ~/case-env
   source ~/case-env/bin/activate
   pip install pyserial sounddevice numpy sphn websockets openwakeword
   ```

### Phase 3: Wire the UART Bridge (Day 3)

7. **Connect three wires**
   - Pi GPIO 14 (TX) -> ESP32 GPIO 3 (RX0)
   - Pi GPIO 15 (RX) <- ESP32 GPIO 1 (TX0)
   - Pi GND <-> ESP32 GND

8. **Test UART link**
   ```bash
   # On Pi:
   python3 -c "
   import serial
   s = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)
   s.write(b'PING\n')
   print(s.readline())
   "
   ```
   ESP32 should echo back `PONG`.

### Phase 4: Move Audio to Pi (Day 3-4)

9. **Wire INMP441 and MAX98357A to Pi GPIO**
   - See pin table in Section 2 and circuit diagram in Section 3

10. **Test audio capture and playback**
    ```bash
    arecord -D plughw:1,0 -f S16_LE -r 24000 -c 1 -d 3 test.wav
    aplay -D plughw:1,0 test.wav
    ```

### Phase 5: Implement Pi Voice Client (Day 4-5)

11. **Create `case_brain.py`** -- the main Pi-side service
    - Opens UART to ESP32
    - Opens I2S mic/speaker via `sounddevice`
    - Runs wake-word detection on mic stream
    - On wake word: opens WebSocket to Moshi directly
      ```
      ws://moshi-host:8998/api/chat?text_prompt=...&voice_prompt=...
      ```
    - Streams Opus audio frames (kind tag `0x01`) bidirectionally
    - Parses text responses (kind tag `0x02`) for motor commands
    - Sends motor commands to ESP32 over UART

12. **Eliminate `personaplex_server.py`**
    - The Pi connects to Moshi WebSocket directly
    - No more HTTP POST proxy, no more PCM-to-Opus bridge
    - The Colab notebook only needs to run Moshi itself

### Phase 6: Integration Testing (Day 5-7)

13. **End-to-end test**
    - Say "Hey CASE" -> wake-word triggers on Pi
    - Speak command -> Pi streams to Moshi via WebSocket
    - Moshi responds -> Pi plays audio + parses intent
    - Motor command -> Pi sends over UART -> ESP32 executes
    - Balance maintained throughout

14. **Stress test**
    - Continuous conversation while balancing
    - Pi process kill -> verify ESP32 keeps balancing
    - WiFi disconnect -> verify graceful degradation

15. **Create systemd services**
    ```ini
    # /etc/systemd/system/case-brain.service
    [Unit]
    Description=CASE Brain (Voice AI + UART)
    After=network-online.target sound.target

    [Service]
    ExecStart=/home/pi/case-env/bin/python /home/pi/case_brain.py
    Restart=always
    RestartSec=3

    [Install]
    WantedBy=multi-user.target
    ```

---

## 10. Risk Analysis

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| UART frame corruption | Medium | Low | Checksum in protocol; re-send on NAK |
| Pi SD card corruption on power loss | Medium | High | Read-only root FS (`overlayfs`); log to tmpfs |
| MPU6050 I2C hang | Low | High | ESP32 watchdog timer + I2C bus recovery |
| Moshi WebSocket disconnect | Medium | Low | Auto-reconnect with exponential backoff; ESP32 unaffected |
| 5V buck converter noise | Low | Medium | LC filter on output; keep away from IMU |
| Pi thermal throttle | Low | Medium | Heatsink + passive airflow from robot motion |
| UART buffer overflow | Low | Low | Flow control via protocol ACK; 115200 baud is 14.4 KB/s (sufficient for commands) |
| Audio feedback (mic picks up speaker) | Medium | Medium | Software echo cancellation or physical separation |
| Pi boot time (15-30 s) | Certain | Low | ESP32 balances immediately; Pi joins when ready |
| Power supply insufficient | Low | High | Size buck converter for 3A (Pi=2.5A peak + peripherals) |

---

## 11. UART Protocol Specification (Pi <-> ESP32)

### Physical Layer
- Baud rate: 115200
- Format: 8N1 (8 data bits, no parity, 1 stop bit)
- Voltage: 3.3 V (both sides, no level shifter)
- Pins: Pi GPIO 14/15 <-> ESP32 GPIO 1/3

### Frame Format

All messages are newline-terminated ASCII for easy debugging via Serial Monitor.

```
<CMD>[:<PARAM>]\n
```

### Pi -> ESP32 Commands

| Command | Format | Description |
|---------|--------|-------------|
| PING | `PING\n` | Heartbeat / link check |
| MODE | `MODE:<mode>\n` | Set motor mode |
| PID | `PID:<Kp>,<Ki>,<Kd>\n` | Update PID constants |
| SETPOINT | `SETPOINT:<value>\n` | Update balance setpoint |
| ESTOP | `ESTOP\n` | Emergency stop (disable motors) |

#### Motor Modes

| Mode string | MotorMode enum | Behavior |
|-------------|---------------|----------|
| `BALANCE` | BALANCE_ONLY | Self-balance in place |
| `FORWARD` | FORWARD_ASSIST | Lean forward + balance |
| `BACKWARD` | BACKWARD_ASSIST | Lean backward + balance |
| `LEFT` | TURN_LEFT | Differential steering left |
| `RIGHT` | TURN_RIGHT | Differential steering right |
| `STOP` | STOPPED | Brake motors |

### ESP32 -> Pi Responses

| Response | Format | Description |
|----------|--------|-------------|
| PONG | `PONG\n` | Heartbeat reply |
| ACK | `ACK:<cmd>\n` | Command accepted |
| NAK | `NAK:<cmd>:<reason>\n` | Command rejected |
| TELEM | `TELEM:<angle>,<output>,<mode>\n` | Telemetry (10 Hz) |
| FAULT | `FAULT:<code>\n` | Error condition |

#### Fault Codes

| Code | Meaning |
|------|---------|
| `DMP_FAIL` | DMP initialization failed |
| `FIFO_OVF` | FIFO overflow (missed reads) |
| `I2C_ERR` | I2C communication error |
| `MOTOR_OC` | Over-current detected (if sensed) |

### Example Exchange

```
Pi  -> ESP32:  PING\n
ESP32 -> Pi:   PONG\n

Pi  -> ESP32:  MODE:FORWARD\n
ESP32 -> Pi:   ACK:MODE\n
ESP32 -> Pi:   TELEM:182.3,45.2,FORWARD\n
ESP32 -> Pi:   TELEM:181.8,42.1,FORWARD\n

Pi  -> ESP32:  MODE:BALANCE\n
ESP32 -> Pi:   ACK:MODE\n

Pi  -> ESP32:  ESTOP\n
ESP32 -> Pi:   ACK:ESTOP\n
```

---

## 12. GPIO Conflict Verification (Option A)

No conflicts exist in Option A. Each pin is used by exactly one function:

**ESP32 pins used:**
- I2C: GPIO 21 (SDA), 22 (SCL)
- IMU interrupt: GPIO 2
- Motor A: GPIO 25 (PWM), 26 (IN1), 27 (IN2)
- Motor B: GPIO 14 (PWM), 12 (IN1), 13 (IN2)
- Motor standby: GPIO 33
- UART: GPIO 1 (TX0), 3 (RX0)

**Pi pins used:**
- UART: GPIO 14 (TX), 15 (RX)
- I2S: GPIO 18 (BCLK), 19 (LRCLK), 20 (DIN), 21 (DOUT)

No overlapping functions. I2S and UART use separate GPIO banks on the Pi.
Motor and IMU pins on ESP32 are unchanged from `self_balance.ino`.
UART uses ESP32's default Serial0 pins (GPIO 1/3), which do not conflict
with any motor or I2C pin.
