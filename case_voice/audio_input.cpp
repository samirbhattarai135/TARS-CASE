#include "audio_input.h"

// Pin definition for analog microphone module
// Connect AO (Analog Out) pin to GPIO 34
// GPIO 34 is ADC1_CH6 and supports analog input

AudioInput::AudioInput() {
    initialized = false;
    dcOffset = 2048;  // Midpoint of 12-bit ADC (0-4095)
    timer = NULL;
}

bool AudioInput::begin() {
    // Configure ADC for analog microphone
    pinMode(ANALOG_MIC_PIN, INPUT);

    // Set ADC resolution to 12 bits (0-4095)
    analogReadResolution(MIC_ADC_RESOLUTION);

    // Set ADC attenuation for full range (0-3.3V)
    // ADC_11db gives 0-3.3V range
    analogSetAttenuation(ADC_11db);

    // Calibrate DC offset (microphone bias voltage)
    Serial.println("Calibrating analog microphone DC offset...");
    calibrateDCOffset();
    Serial.printf("DC offset: %d (ADC units)\n", dcOffset);

    Serial.println("Audio input initialized (Analog microphone)");
    Serial.printf("Microphone pin: GPIO %d\n", ANALOG_MIC_PIN);
    Serial.println("Note: Analog mic has lower quality than I2S.");
    Serial.println("Consider upgrading to INMP441 for better audio.");

    initialized = true;
    return true;
}

void AudioInput::calibrateDCOffset() {
    // Read multiple samples to determine DC bias
    const int numSamples = 1000;
    int32_t sum = 0;

    for (int i = 0; i < numSamples; i++) {
        sum += analogRead(ANALOG_MIC_PIN);
        delayMicroseconds(100);  // Small delay between readings
    }

    dcOffset = sum / numSamples;
}

uint16_t AudioInput::readRaw() {
    if (!initialized) return 0;
    return analogRead(ANALOG_MIC_PIN);
}

size_t AudioInput::read(int16_t* buffer, size_t numSamples) {
    if (!initialized) return 0;

    // Calculate delay between samples for desired sample rate
    // For 16kHz: 1000000 microseconds / 16000 samples = 62.5 us per sample
    const uint32_t samplePeriodUs = 1000000 / MIC_SAMPLE_RATE;

    for (size_t i = 0; i < numSamples; i++) {
        // Read ADC value (0-4095 for 12-bit)
        uint16_t adcValue = analogRead(ANALOG_MIC_PIN);

        // Remove DC offset and convert to signed 16-bit
        // ADC range: 0-4095 (12-bit)
        // Convert to: -32768 to +32767 (16-bit signed)
        int32_t centered = (int32_t)adcValue - dcOffset;

        // Scale from 12-bit to 16-bit range
        // Multiply by 16 to use full 16-bit range (2^16 / 2^12 = 16)
        buffer[i] = (int16_t)(centered * 16);

        // Wait for next sample period
        delayMicroseconds(samplePeriodUs);
    }

    return numSamples;
}

bool AudioInput::isAvailable() {
    return initialized;
}
