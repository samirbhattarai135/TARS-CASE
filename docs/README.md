# CASE - Voice-Controlled Self-Balancing Robot

An AI-powered self-balancing robot inspired by CASE from Interstellar, featuring voice control via Nvidia Personaplex and dual-core ESP32 architecture.

## Overview

CASE combines:
- **Self-balancing**: MPU6050 IMU + PID control on ESP32
- **Voice control**: On-device wake word detection + cloud AI conversations
- **Hybrid AI**: Local motor commands (<50ms) + Colab Personaplex for complex queries
- **Dual-core architecture**: Balance control never blocked by AI processing

## Features

- ✅ Self-balancing on two wheels (100Hz PID control)
- 🎙️ Voice commands: "Hey CASE, move forward/backward/turn left/right/stop"
- 🤖 AI conversations via Nvidia Personaplex on Google Colab
- 🔊 Audio feedback and responses
- 🧠 FreeRTOS dual-core task management (ESP32)

## Hardware

### Required Components

| Component | Model | Purpose | Approx. Cost |
|-----------|-------|---------|--------------|
| Microcontroller | ESP32-S3 or ESP32-WROVER | Main controller with PSRAM | $8-12 |
| IMU | MPU6050 | Balance sensing (I2C) | $5 |
| Motor Driver | TB6612FNG | Dual motor control | $6 |
| Motors | DC Geared Motors (2x) | Drive wheels | $10 |
| Microphone | INMP441 | Voice input (I2S) | $4 |
| Amplifier | MAX98357A | Audio output (I2S) | $5 |
| Speaker | 4Ω 3W | Voice responses | $3 |
| Battery | LiPo 7.4V 2S | Power supply | $15 |
| Chassis | Custom or kit | Robot frame | $20 |

**Total: ~$86**

### Pin Connections

#### Existing (Self-Balancing)
- **MPU6050 IMU**: SDA→21, SCL→22, INT→2
- **TB6612FNG Motor Driver**:
  - Motor A: PWM→25, IN1→26, IN2→27
  - Motor B: PWM→14, IN1→12, IN2→13
  - Standby→33

#### New (Voice Control)
- **INMP441 Microphone**: SCK→32, WS→15, SD→4
- **MAX98357A Amplifier**: BCLK→18, LRC→19, DIN→23

See `setup_guide.md` for detailed wiring instructions.

## Software Architecture

```
┌─────────────────────── ESP32 ───────────────────────┐
│                                                      │
│  Core 1 (High Priority)    Core 0 (Normal Priority) │
│  ┌──────────────────┐    ┌─────────────────────┐   │
│  │ Balance Control  │    │ Audio Processing    │   │
│  │  - MPU6050 Read  │    │  - Wake Word Detect │   │
│  │  - PID Compute   │    │  - Command Classify │   │
│  │  - Motor Control │◄───│  - Colab Streaming  │   │
│  │  (~100Hz loop)   │    │  - Audio Playback   │   │
│  └──────────────────┘    └─────────────────────┘   │
│                                   │                 │
└───────────────────────────────────┼─────────────────┘
                                    │
                              WiFi WebSocket
                                    │
                            ┌───────▼────────┐
                            │ Google Colab   │
                            │  Personaplex   │
                            │ (Nvidia Moshi) │
                            └────────────────┘
```

## Quick Start

1. **Build the hardware** - Follow `setup_guide.md`
2. **Upload balance code** - Test with `self_balance/self_balance.ino`
3. **Add audio hardware** - Wire INMP441 and MAX98357A
4. **Upload voice code** - Flash `case_voice/case_voice.ino`
5. **Setup Colab server** - Run `colab/setup_colab.ipynb`
6. **Test** - Say "Hey CASE" and watch it respond!

## Implementation Status

### ✅ Phase 1-2: Foundation (COMPLETE)
- [x] Dual-core FreeRTOS architecture
- [x] Balance control module extracted
- [x] I2S audio input/output interfaces
- [x] Wake word detector (placeholder)
- [x] Command classifier (placeholder)
- [x] Colab client (placeholder)
- [x] Main sketch with state machine

### 🟡 Phase 3-8: To Do
- [ ] Wake word ML model (TFLite)
- [ ] Command classification
- [ ] Personaplex integration
- [ ] WebSocket streaming
- [ ] Full testing

## Documentation

- **[Setup Guide](setup_guide.md)** - Complete build instructions
- **[Troubleshooting](troubleshooting.md)** - Common issues and fixes
- **[Wiring Diagram](wiring_diagram.md)** - Pin connections and schematics

## License

MIT License - See LICENSE file

## Acknowledgments

- Inspired by CASE from *Interstellar* (2014)
- Built with Nvidia Personaplex (Moshi framework)
- MPU6050 library by Jeff Rowberg
- PID library by Brett Beauregard

---

**Status:** Phase 1-2 Complete | Ready for hardware assembly and testing
