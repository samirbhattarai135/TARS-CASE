# CASE Voice Control Firmware

ESP32 firmware for voice-controlled self-balancing robot with Nvidia Personaplex AI integration.

## Architecture

### Dual-Core FreeRTOS Design

**Core 1 (High Priority):**

- Balance control loop (~100Hz)
- MPU6050 IMU reading
- PID computation
- Motor control with voice command overlay

**Core 0 (Normal Priority):**

- Audio input buffering (I2S microphone)
- Wake word detection
- Voice command classification
- WebSocket streaming to Colab Personaplex
- Audio output playback (I2S speaker)

### State Machine

```
IDLE → (wake word) → LISTENING → (classify) → PROCESSING_LOCAL → IDLE
                                            └→ PROCESSING_COLAB (streams reply to speaker) → IDLE
```

## Pin Configuration

See `docs/wiring_diagram.md` for complete pinout.

**Audio (New):**

- I2S Mic (INMP441): SCK=32, WS=15, SD=4
- I2S Speaker (MAX98357A): BCLK=26, LRCK=25, DIN=27

**Balance (Existing):**

- MPU6050: SDA=21, SCL=22, INT=2
- Motors: See TB6612FNG connections

## Configuration

**WiFi Credentials** (top of `case_voice.ino`):

```cpp
const char* WIFI_SSID = "YourNetwork";
const char* WIFI_PASSWORD = "YourPassword";
```

**Colab Server URL** — the Cloudflare tunnel to `colab/personaplex_server.py` (port 9001), NOT directly to Moshi. Update it each time you restart the tunnel:

```cpp
const char* COLAB_SERVER_URL = "https://your-tunnel-url.trycloudflare.com";
```

## Modules

| File                   | Purpose                         | Status         |
| ---------------------- | ------------------------------- | -------------- |
| `balance_control.*`    | Self-balancing PID control      | ✅ Complete    |
| `audio_input.*`        | I2S microphone interface        | ✅ Complete    |
| `audio_output.*`       | I2S speaker + amplifier         | ✅ Complete    |
| `wake_word.*`          | Wake word detection             | ⚠️ Placeholder |
| `command_classifier.*` | Voice command classification    | ⚠️ Placeholder |
| `colab_client.*`       | HTTP client to Personaplex bridge | ✅ Complete  |

## Dependencies

Install via Arduino Library Manager:

- I2Cdev
- MPU6050
- PID_v1

Built-in (no install needed):

- ESP32-I2S
- FreeRTOS
- WiFi / HTTPClient (used for the Personaplex bridge)

## Upload Instructions

1. Install ESP32 board support in Arduino IDE
2. Select board: **ESP32 Dev Module** or **ESP32-S3**
3. Configure WiFi credentials above
4. Upload `case_voice.ino`
5. Open Serial Monitor (115200 baud)

## Expected Output

```
=== CASE Voice-Controlled Robot ===
Initializing systems...

Initializing MPU6050...
MPU6050 connection successful
DMP ready!

Initializing audio systems...
Audio input initialized
Audio output initialized

Initializing network...
WiFi connected!
IP address: 192.168.1.100

Starting dual-core tasks...
[Core 1] Balance task started
[Core 0] Audio task started

=== System Ready ===
Say 'Hey CASE' to activate voice control
```

Startup beeps: 1000Hz (200ms), 1500Hz (200ms)

## Testing

**Balance Test:**

- Place robot upright
- Should self-balance and maintain position

**Audio Test:**

- Speak loudly or clap
- Should see `[Audio] Wake word detected!` in serial
- Should hear confirmation beep (2000Hz)

**WiFi Test:**

- Check serial for "WiFi connected!"
- Should show IP address

## Current Limitations

**Current Status:**

- Wake word detection is placeholder (triggers on any loud sound)
- Command classification not implemented (always returns CMD_COMPLEX, so every
  utterance goes to the Colab bridge)
- Colab path is complete: HTTP POST to `colab/personaplex_server.py`, reply
  streamed to the speaker

**Next Steps:**

- Phase 3: Train TFLite wake word model
- Phase 4: Implement command classifier
- Later: replace the fixed 1.5 s recording window with silence-based endpointing

## Troubleshooting

**Robot falls:**

- Check serial for "DMP ready!" (MPU6050 working)
- Verify balance task running: `[Core 1] Balance task started`
- Tune PID in `balance_control.cpp`

**No audio:**

- Check pins: Mic SD=4, Speaker DIN=27
- Verify power: Mic 3.3V, Speaker 5V
- Look for "Audio input/output initialized"

**No WiFi:**

- Check credentials spelling
- Ensure 2.4GHz network (not 5GHz)
- ESP32 may need more connection attempts

See `docs/troubleshooting.md` for detailed solutions.

## Performance

**Target Specs:**

- Balance loop: 100Hz (10ms period)
- Wake word latency: <100ms
- Local command response: <50ms
- Colab query round-trip: <3s (network dependent)

**Memory:**

- Recommend ESP32-S3 with PSRAM for audio buffering
- Basic ESP32 may work with reduced buffer sizes

## Code Quality

- ✅ Modular architecture
- ✅ Separation of concerns
- ✅ Error handling
- ✅ Graceful degradation
- ✅ Well-documented functions
- ✅ FreeRTOS best practices

## Contributing

This is a capstone project. Code is provided as reference implementation.

## License

MIT License - See root LICENSE file

┌──────┬───────────────────┬────────────────────────────┐
│ GPIO │ Function │ Component │
├──────┼───────────────────┼────────────────────────────┤
│ 2 │ MPU INT │ MPU6050 │
├──────┼───────────────────┼────────────────────────────┤
│ 4 │ I2S SD (mic data) │ INMP441 │
├──────┼───────────────────┼────────────────────────────┤
│ 12 │ BIN1 │ TB6612FNG Motor B │
├──────┼───────────────────┼────────────────────────────┤
│ 13 │ BIN2 │ TB6612FNG Motor B │
├──────┼───────────────────┼────────────────────────────┤
│ 14 │ PWMB │ TB6612FNG Motor B │
├──────┼───────────────────┼────────────────────────────┤
│ 16 │ PWMA │ TB6612FNG Motor A (was 25) │
├──────┼───────────────────┼────────────────────────────┤
│ 17 │ AIN1 │ TB6612FNG Motor A (was 26) │
├──────┼───────────────────┼────────────────────────────┤
│ 19 │ AIN2 │ TB6612FNG Motor A (was 27) │
├──────┼───────────────────┼────────────────────────────┤
│ 15 │ I2S WS │ INMP441 │
├──────┼───────────────────┼────────────────────────────┤
│ 21 │ SDA │ MPU6050 │
├──────┼───────────────────┼────────────────────────────┤
│ 22 │ SCL │ MPU6050 │
├──────┼───────────────────┼────────────────────────────┤
│ 25 │ I2S LRCK │ MAX98357A speaker │
├──────┼───────────────────┼────────────────────────────┤
│ 26 │ I2S BCK │ MAX98357A speaker │
├──────┼───────────────────┼────────────────────────────┤
│ 27 │ I2S DIN │ MAX98357A speaker │
├──────┼───────────────────┼────────────────────────────┤
│ 32 │ I2S SCK │ INMP441 │
├──────┼───────────────────┼────────────────────────────┤
│ 33 │ STBY │ TB6612FNG │
└──────┴───────────────────┴────────────────────────────┘
=
