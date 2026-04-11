#ifndef WAKE_WORD_H
#define WAKE_WORD_H

#include <Arduino.h>

// Wake word detection using simple keyword spotting
// TODO: Replace with TFLite model in Phase 3
class WakeWordDetector {
public:
    WakeWordDetector();
    bool begin();
    bool detect(int16_t* audioBuffer, size_t numSamples);
    float getConfidence();

private:
    bool initialized;
    float confidence;

    // Simple energy-based detection (placeholder)
    float calculateEnergy(int16_t* buffer, size_t numSamples);
};

#endif
