# CASE Wiring Diagram

Complete pin connections for the voice-controlled self-balancing robot.

## Pin Assignment Table

| Function | GPIO | Component | Direction |
|----------|------|-----------|-----------|
| **I2C Bus** |
| I2C SDA | 21 | MPU6050 | Bidirectional |
| I2C SCL | 22 | MPU6050 | Output |
| **MPU6050 Interrupt** |
| MPU INT | 2 | MPU6050 | Input |
| **Motor Driver A** |
| Motor A PWM | 25 | TB6612FNG PWMA | Output (PWM) |
| Motor A IN1 | 26 | TB6612FNG AIN1 | Output |
| Motor A IN2 | 27 | TB6612FNG AIN2 | Output |
| **Motor Driver B** |
| Motor B PWM | 14 | TB6612FNG PWMB | Output (PWM) |
| Motor B IN1 | 12 | TB6612FNG BIN1 | Output |
| Motor B IN2 | 13 | TB6612FNG BIN2 | Output |
| **Motor Driver Control** |
| Standby | 33 | TB6612FNG STBY | Output |
| **Analog Microphone** |
| Analog Mic | 34 | Analog Mic AO | Input (ADC) |
| **I2S Speaker** |
| I2S Spk BCLK | 18 | MAX98357A BCLK | Output |
| I2S Spk LRC | 19 | MAX98357A LRC | Output |
| I2S Spk DIN | 23 | MAX98357A DIN | Output |

## Connection Diagrams

### MPU6050 IMU

```
MPU6050          ESP32
┌─────────┐
│ VCC     ├──────→ 3.3V
│ GND     ├──────→ GND
│ SDA     ├──────→ GPIO 21 (I2C SDA)
│ SCL     ├──────→ GPIO 22 (I2C SCL)
│ INT     ├──────→ GPIO 2  (Interrupt)
│ AD0     ├──────→ GND (I2C address 0x68)
└─────────┘
```

### TB6612FNG Motor Driver

```
TB6612FNG        ESP32         Other
┌──────────┐
│ VM       ├─────────────────→ Battery+ (7.4V)
│ VCC      ├──────→ 3.3V
│ GND      ├──────→ GND ──────→ Battery−
│          │
│ PWMA     ├──────→ GPIO 25
│ AIN1     ├──────→ GPIO 26
│ AIN2     ├──────→ GPIO 27
│          │
│ PWMB     ├──────→ GPIO 14
│ BIN1     ├──────→ GPIO 12
│ BIN2     ├──────→ GPIO 13
│          │
│ STBY     ├──────→ GPIO 33 (must be HIGH to enable)
│          │
│ AO1      ├─────────────────→ Motor A (Left) +
│ AO2      ├─────────────────→ Motor A (Left) −
│          │
│ BO1      ├─────────────────→ Motor B (Right) +
│ BO2      ├─────────────────→ Motor B (Right) −
└──────────┘
```

**Notes:**
- VM powers the motors (7.4V from battery)
- VCC powers logic (3.3V from ESP32)
- Ground MUST be common between ESP32, TB6612FNG, and battery

### Analog Microphone Module (Your Hardware)

```
Analog Mic       ESP32
┌──────────┐
│ VCC      ├──────→ 3.3V
│ GND      ├──────→ GND
│ AO       ├──────→ GPIO 34 (ADC1_CH6)
│ DO       ├──────→ (not used)
└──────────┘
```

**Notes:**
- AO = Analog Out (audio signal)
- DO = Digital Out (threshold comparator - not needed)
- Blue potentiometer adjusts microphone gain
- GPIO 34 is input-only, perfect for analog input

**Alternative: INMP441 I2S Microphone (Future Upgrade)**

If you later upgrade to a digital I2S microphone:
```
INMP441          ESP32
┌──────────┐
│ VDD      ├──────→ 3.3V
│ GND      ├──────→ GND
│ SD       ├──────→ GPIO 4  (I2S data)
│ WS       ├──────→ GPIO 15 (word select)
│ SCK      ├──────→ GPIO 32 (clock)
│ L/R      ├──────→ GND (left channel)
└──────────┘
```
Benefits: Better audio quality for Personaplex conversations

### MAX98357A I2S Amplifier + Speaker

```
MAX98357A        ESP32         Other
┌──────────┐
│ VIN      ├──────→ 5V (or VBAT if >5V)
│ GND      ├──────→ GND
│          │
│ DIN      ├──────→ GPIO 23 (I2S data)
│ BCLK     ├──────→ GPIO 18 (bit clock)
│ LRC      ├──────→ GPIO 19 (word select)
│          │
│ GAIN     ├──────→ GND (9dB gain, can float for 15dB)
│ SD       ├──────→ VIN (shutdown control, VIN = always on)
│          │
│ +        ├─────────────────→ Speaker + (4Ω 3W)
│ −        ├─────────────────→ Speaker −
└──────────┘
```

**Notes:**
- VIN can be 3.3V-5.5V (5V recommended for more volume)
- GAIN pin controls amplification:
  - GND = 9dB
  - Float = 15dB (default)
  - VIN = 6dB
- SD (shutdown) tied to VIN keeps amplifier always on
- Speaker should be 4Ω or 8Ω, 3W rating

## Power Distribution

```
┌──────────────┐
│ 7.4V LiPo    │
│ Battery      │
│ (2S 2000mAh) │
└───┬──────┬───┘
    │      │
    │      └──→ TB6612FNG VM (motor power)
    │
    ├──→ Buck Converter (7.4V → 5V)
    │    └──→ ESP32 VIN
    │         └──→ MAX98357A VIN
    │
    └──→ GND (common ground for all components)
```

**Alternative (if ESP32 has 3.3V regulator):**
```
Battery 7.4V → ESP32 VIN → ESP32 3.3V pin → MPU6050, INMP441
Battery 7.4V → TB6612FNG VM
Battery 7.4V → Buck 5V → MAX98357A VIN
```

## Physical Layout Recommendations

### Top View (Component Placement)
```
        ┌─────────────────┐
        │   MPU6050 IMU   │ ← Mount centered, above wheel axis
        │  (on top)       │
        └─────────────────┘
              ╱│╲
             ╱ │ ╲ Short wires
            ╱  │  ╲
    ┌──────────────────┐
    │                  │
    │   ESP32-S3       │ ← Main controller
    │                  │
    ├──────────────────┤
    │  TB6612FNG       │ ← Motor driver
    └──────────────────┘
       │         │
       Motor A   Motor B
       ╱           ╲
      O             O   ← Wheels
```

### Component Mounting Tips

**MPU6050:**
- Mount rigidly to chassis
- Position above wheel axis for best balance
- Keep away from motors (magnetic interference)
- Use vibration damping foam if motors cause oscillation

**INMP441 Microphone:**
- Mount facing forward/upward
- Away from motors and speaker (reduce noise)
- Consider foam windscreen for outdoor use

**MAX98357A + Speaker:**
- Mount speaker facing outward
- Secure speaker with screws (not glue - vibrations)
- Keep amplifier module away from microphone

**ESP32 + TB6612FNG:**
- Stack close together (short wiring)
- Both need access to battery ground
- TB6612FNG may need heatsink if motors draw >1A

## Wire Gauge Recommendations

| Connection | Wire Gauge | Notes |
|------------|------------|-------|
| Battery to TB6612FNG VM | 20-22 AWG | High current |
| Motor outputs | 22-24 AWG | Medium current |
| ESP32 power | 24-26 AWG | Low current OK |
| Signal wires (GPIO) | 26-28 AWG | Thin is fine |
| I2C bus (SDA/SCL) | 26-28 AWG | Keep short (<20cm) |
| I2S bus | 26-28 AWG | Can be longer |

## Common Mistakes to Avoid

### ❌ Wrong
- **Separate grounds** - ESP32 and battery on different ground planes
- **5V to MPU6050** - Will damage sensor (3.3V only!)
- **Missing pullup resistors on I2C** - Usually built-in, but check if bus errors occur
- **INMP441 L/R floating** - Must connect to GND or VDD

### ✅ Correct
- **Common ground** - All GND pins connected
- **3.3V logic** - MPU6050, INMP441 on 3.3V
- **Short I2C wires** - Under 20cm for reliable communication
- **Twisted pairs** - For motor wires (reduces EMI)

## Testing Checklist

Before powering on:
- [ ] All grounds connected (ESP32, TB6612FNG, battery, sensors)
- [ ] No 5V going to 3.3V components
- [ ] Motor wires not shorted
- [ ] Battery polarity correct
- [ ] I2C pullup resistors present (10kΩ typical, check MPU6050 module)
- [ ] No loose wires touching multiple pins

## Troubleshooting

### I2C Not Working
- Check SDA/SCL not swapped
- Verify 3.3V power to MPU6050
- Try external 4.7kΩ pullup resistors on SDA/SCL to 3.3V

### Motors Not Running
- Check STBY pin is HIGH (GPIO 33)
- Verify VM has battery voltage (~7.4V)
- Test motor directly with battery (should spin)
- Check GND connection

### Microphone No Input
- Verify L/R pin connected (GND for left channel)
- Check VDD is 3.3V
- Test with clapping (should see serial output)

### Speaker No Output
- Check VIN voltage (needs 3.3V-5V)
- Verify speaker connected (measure resistance: ~4Ω or ~8Ω)
- Try different GAIN setting
- Ensure SD pin is not LOW (would shut down)

---

**Safety Warning:**
- Never reverse battery polarity
- Use fuse on battery positive lead (recommended: 3A fast-blow)
- Disconnect battery when uploading code
- Watch for overheating components
