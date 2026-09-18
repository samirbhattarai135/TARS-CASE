#ifndef AUDIO_OUTPUT_H
#define AUDIO_OUTPUT_H

#include <Arduino.h>
#include "i2s_bus.h"

// The MAX98357A's clock and data pins, its sample rate and its slot format
// all come from i2s_bus.h, because the microphone shares them.

// SD_MODE. Driven high the amplifier runs and averages the two slots; driven
// low it shuts down, which is how the robot stays quiet without the I2S port
// having to stop.
#define AMP_SD_PIN        GPIO_NUM_4

#define SPK_SAMPLE_RATE   I2S_SAMPLE_RATE
#define SPK_BUFFER_SIZE   512       // Mono samples per write

class AudioOutput {
public:
    AudioOutput();
    bool begin();
    size_t write(const int16_t* buffer, size_t numSamples);
    void playTone(uint16_t frequency, uint16_t durationMs);
    // Powers the amplifier stage. Muting here is silent in a way that writing
    // zero samples is not, because the class-D output stage stops switching.
    void setEnabled(bool enabled);
    bool isAvailable();

private:
    bool initialized;
};

#endif
