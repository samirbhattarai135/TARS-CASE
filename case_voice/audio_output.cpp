#include "audio_output.h"
#include <math.h>

// One stereo frame is two 32-bit slots, both carrying the same sample.
static int32_t stereoBuf[SPK_BUFFER_SIZE * 2];

AudioOutput::AudioOutput() {
    initialized = false;
}

bool AudioOutput::begin() {
    // Hold the amplifier off until the port is running, so it cannot squawk
    // at whatever the I2S lines happen to be doing during setup.
    pinMode(AMP_SD_PIN, OUTPUT);
    digitalWrite(AMP_SD_PIN, LOW);

    if (!i2sBusBegin()) return false;

    digitalWrite(AMP_SD_PIN, HIGH);

    Serial.println("OK: Speaker  (MAX98357A — shared I2S, data on GPIO 23)");
    initialized = true;
    return true;
}

void AudioOutput::setEnabled(bool enabled) {
    digitalWrite(AMP_SD_PIN, enabled ? HIGH : LOW);
}

size_t AudioOutput::write(const int16_t* buffer, size_t numSamples) {
    if (!initialized || numSamples == 0) return 0;

    // The same sample goes to both slots. The MAX98357A is strapped to average
    // them, so the average is the sample. 16-bit audio is left-aligned into
    // the 32-bit slot the shared clock format dictates.
    size_t chunk = min(numSamples, (size_t)SPK_BUFFER_SIZE);
    for (size_t i = 0; i < chunk; i++) {
        int32_t sample = (int32_t)buffer[i] << 16;
        stereoBuf[i * 2]     = sample;   // Left
        stereoBuf[i * 2 + 1] = sample;   // Right
    }

    const size_t frameBytes = 2 * sizeof(int32_t);
    size_t bytesWritten = 0;
    i2s_channel_write(i2sBusTx(), stereoBuf, chunk * frameBytes,
                      &bytesWritten, portMAX_DELAY);
    return bytesWritten / frameBytes;
}

void AudioOutput::playTone(uint16_t frequency, uint16_t durationMs) {
    if (!initialized) return;

    const int totalSamples = (SPK_SAMPLE_RATE * durationMs) / 1000;
    int16_t mono[128];

    for (int i = 0; i < totalSamples; i += 128) {
        int chunkSize = min(128, totalSamples - i);
        for (int j = 0; j < chunkSize; j++) {
            float t = (float)(i + j) / SPK_SAMPLE_RATE;
            mono[j] = (int16_t)(sinf(2.0f * M_PI * frequency * t) * 8000);
        }
        write(mono, chunkSize);
    }
}

bool AudioOutput::isAvailable() {
    return initialized;
}
