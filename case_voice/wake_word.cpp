#include "wake_word.h"

WakeWordDetector::WakeWordDetector() {
    initialized = false;
    confidence = 0.0;
}

bool WakeWordDetector::begin() {
    Serial.println("Wake word detector initialized (placeholder)");
    Serial.println("TODO: Load TFLite model for 'Hey CASE' detection");
    initialized = true;
    return true;
}

bool WakeWordDetector::detect(int16_t* audioBuffer, size_t numSamples) {
    if (!initialized) return false;

    // PLACEHOLDER: Simple energy-based detection
    // TODO: Replace with TFLite Micro model in Phase 3
    float energy = calculateEnergy(audioBuffer, numSamples);

    // Very basic threshold (will trigger on any loud sound)
    const float ENERGY_THRESHOLD = 5000000.0;

    if (energy > ENERGY_THRESHOLD) {
        confidence = 0.8; // Fake confidence
        return true;
    }

    confidence = 0.0;
    return false;
}

float WakeWordDetector::calculateEnergy(int16_t* buffer, size_t numSamples) {
    float energy = 0.0;
    for (size_t i = 0; i < numSamples; i++) {
        energy += buffer[i] * buffer[i];
    }
    return energy / numSamples;
}

float WakeWordDetector::getConfidence() {
    return confidence;
}
