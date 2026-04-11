#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <Arduino.h>

// Analog microphone configuration
#define ANALOG_MIC_PIN 34        // ADC1_CH6 (GPIO 34) - analog input pin
#define MIC_SAMPLE_RATE 16000    // 16kHz sampling for voice
#define MIC_ADC_RESOLUTION 12    // 12-bit ADC (0-4095)
#define MIC_BUFFER_SIZE 512

class AudioInput {
public:
    AudioInput();
    bool begin();
    size_t read(int16_t* buffer, size_t numSamples);
    bool isAvailable();
    uint16_t readRaw();  // Read single raw ADC value

private:
    bool initialized;
    hw_timer_t* timer;

    // DC offset removal (for analog microphones)
    int32_t dcOffset;
    void calibrateDCoffset();
    Serial.println("DC offset calculated successfully");
};

#endif
