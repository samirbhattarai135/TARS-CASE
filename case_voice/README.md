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
                                            └→ PROCESSING_COLAB → SPEAKING → IDLE
```

## Pin Configuration

See `docs/wiring_diagram.md` for complete pinout.

**Audio (New):**
- I2S Mic: SCK=32, WS=15, SD=4
- I2S Speaker: BCLK=18, LRC=19, DIN=23

**Balance (Existing):**
- MPU6050: SDA=21, SCL=22, INT=2
- Motors: See TB6612FNG connections

## Configuration

**WiFi Credentials** (lines 23-24):
```cpp
const char* WIFI_SSID = "YourNetwork";
const char* WIFI_PASSWORD = "YourPassword";
```

**Colab Server URL** (line 25):
```cpp
const char* COLAB_SERVER_URL = "wss://your-ngrok-url.ngrok.io/audio";
```

## Modules

| File | Purpose | Status |
|------|---------|--------|
| `balance_control.*` | Self-balancing PID control | ✅ Complete |
| `audio_input.*` | I2S microphone interface | ✅ Complete |
| `audio_output.*` | I2S speaker + amplifier | ✅ Complete |
| `wake_word.*` | Wake word detection | ⚠️ Placeholder |
| `command_classifier.*` | Voice command classification | ⚠️ Placeholder |
| `colab_client.*` | WebSocket client to Personaplex | ⚠️ Stub |

## Dependencies

Install via Arduino Library Manager:
- I2Cdev
- MPU6050
- PID_v1

Built-in (no install needed):
- ESP32-I2S
- FreeRTOS

To be added (Phase 6):
- ArduinoWebSockets

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

**Phase 2 Status:**
- Wake word detection is placeholder (triggers on any loud sound)
- Command classification not implemented (always returns CMD_COMPLEX)
- WebSocket client is stub (Colab integration pending)

**Next Steps:**
- Phase 3: Train TFLite wake word model
- Phase 4: Implement command classifier
- Phase 6: Add WebSocket streaming

## Troubleshooting

**Robot falls:**
- Check serial for "DMP ready!" (MPU6050 working)
- Verify balance task running: `[Core 1] Balance task started`
- Tune PID in `balance_control.cpp`

**No audio:**
- Check pins: Mic SD=4, Speaker DIN=23
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
