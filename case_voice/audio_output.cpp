#include "audio_output.h"
#include <math.h>

// Pin definitions for MAX98357A amplifier
#define I2S_SPK_BCLK  18
#define I2S_SPK_LRC   19
#define I2S_SPK_DIN   23

AudioOutput::AudioOutput() {
    initialized = false;
}

bool AudioOutput::begin() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = I2S_SPK_SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_SPK_BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SPK_BCLK,
        .ws_io_num = I2S_SPK_LRC,
        .data_out_num = I2S_SPK_DIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    esp_err_t err = i2s_driver_install(I2S_SPK_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Failed to install I2S speaker driver: %d\n", err);
        return false;
    }

    err = i2s_set_pin(I2S_SPK_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Failed to set I2S speaker pins: %d\n", err);
        return false;
    }

    Serial.println("Audio output initialized");
    initialized = true;
    return true;
}

size_t AudioOutput::write(int16_t* buffer, size_t numSamples) {
    if (!initialized) return 0;

    size_t bytesWritten = 0;
    i2s_write(I2S_SPK_PORT, buffer, numSamples * sizeof(int16_t), &bytesWritten, portMAX_DELAY);

    return bytesWritten / sizeof(int16_t);
}

void AudioOutput::playTone(uint16_t frequency, uint16_t durationMs) {
    if (!initialized) return;

    const int numSamples = (I2S_SPK_SAMPLE_RATE * durationMs) / 1000;
    int16_t buffer[128];

    for (int i = 0; i < numSamples; i += 128) {
        int chunkSize = min(128, numSamples - i);

        for (int j = 0; j < chunkSize; j++) {
            float t = (float)(i + j) / I2S_SPK_SAMPLE_RATE;
            buffer[j] = (int16_t)(sin(2 * PI * frequency * t) * 8000);
        }

        write(buffer, chunkSize);
    }
}

bool AudioOutput::isAvailable() {
    return initialized;
}
