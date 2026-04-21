#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <Arduino.h>

// INMP441 I2S microphone pin configuration
#define I2S_MIC_SCK   GPIO_NUM_32   // Bit clock
#define I2S_MIC_WS    GPIO_NUM_15   // Word select
#define I2S_MIC_SD    GPIO_NUM_4    // Data from mic

#define MIC_SAMPLE_RATE   16000     // 16 kHz sampling for voice
#define MIC_BUFFER_SIZE   512       // Samples per read

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
