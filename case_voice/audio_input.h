#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <Arduino.h>
#include "i2s_bus.h"

// The INMP441's clock and data pins, its sample rate and its slot format all
// come from i2s_bus.h, because the amplifier shares them.

#define MIC_SAMPLE_RATE   I2S_SAMPLE_RATE
#define MIC_BUFFER_SIZE   512       // Mono samples per read

class AudioInput {
public:
    AudioInput();
    bool begin();
    size_t read(int16_t* buffer, size_t numSamples);
    bool isAvailable();

private:
    bool initialized;
};

#endif
