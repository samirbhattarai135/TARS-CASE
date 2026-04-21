#ifndef AUDIO_OUTPUT_H
#define AUDIO_OUTPUT_H

#include <Arduino.h>

// MAX98357A I2S amplifier pin configuration (I2S0 TX)
#define I2S_SPK_BCK   GPIO_NUM_26
#define I2S_SPK_LRCK  GPIO_NUM_25
#define I2S_SPK_DIN   GPIO_NUM_27

#define SPK_SAMPLE_RATE   16000
#define SPK_BUFFER_SIZE   512

class AudioOutput {
public:
    AudioOutput();
    bool begin();
    size_t write(const int16_t* buffer, size_t numSamples);
    void playTone(uint16_t frequency, uint16_t durationMs);
    bool isAvailable();

private:
    bool initialized;
};

#endif
