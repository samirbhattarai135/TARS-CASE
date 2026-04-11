#ifndef AUDIO_OUTPUT_H
#define AUDIO_OUTPUT_H

#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_SPK_PORT I2S_NUM_1
#define I2S_SPK_SAMPLE_RATE 16000
#define I2S_SPK_BITS_PER_SAMPLE 16
#define I2S_SPK_BUFFER_SIZE 512

class AudioOutput {
public:
    AudioOutput();
    bool begin();
    size_t write(int16_t* buffer, size_t numSamples);
    void playTone(uint16_t frequency, uint16_t durationMs);
    bool isAvailable();

private:
    bool initialized;
};

#endif
