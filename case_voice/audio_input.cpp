#include "audio_input.h"

// One stereo frame is two 32-bit slots, and only the left one carries audio.
static int32_t i2sBuf[MIC_BUFFER_SIZE * 2];

AudioInput::AudioInput() {
    initialized = false;
}

bool AudioInput::begin() {
    if (!i2sBusBegin()) return false;

    Serial.println("OK: Microphone (INMP441 — shared I2S, data on GPIO 34)");
    initialized = true;
    return true;
}

size_t AudioInput::read(int16_t* buffer, size_t numSamples) {
    if (!initialized || numSamples == 0) return 0;

    // Clamp to internal buffer size
    if (numSamples > MIC_BUFFER_SIZE) numSamples = MIC_BUFFER_SIZE;

    size_t bytesRead = 0;
    esp_err_t err = i2s_channel_read(i2sBusRx(), i2sBuf,
                                     numSamples * 2 * sizeof(int32_t),
                                     &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) return 0;

    size_t frames = bytesRead / (2 * sizeof(int32_t));

    // Take the left slot of each frame and convert 32-bit I2S to 16-bit
    // signed. The INMP441 grounds its L/R pin, so the right slot is silent.
    // 24-bit audio sits left-aligned in the slot, and >> 16 keeps the sign
    // bit while landing in int16 range.
    for (size_t i = 0; i < frames; i++) {
        buffer[i] = (int16_t)(i2sBuf[i * 2] >> 16);
    }

    return frames;
}

bool AudioInput::isAvailable() {
    return initialized;
}
