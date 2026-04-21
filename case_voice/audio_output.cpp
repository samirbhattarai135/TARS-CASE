#include "audio_output.h"
#include "driver/i2s_std.h"
#include <math.h>

// I2S0 TX channel — I2S1 is reserved for INMP441 mic input
static i2s_chan_handle_t tx_chan = NULL;

// Stereo buffer for I2S write (MAX98357A expects stereo frames)
static int16_t stereoBuf[SPK_BUFFER_SIZE * 2];

AudioOutput::AudioOutput() {
    initialized = false;
}

bool AudioOutput::begin() {
    // I2S_NUM_0 for TX — audio_input uses I2S_NUM_1 for RX
    i2s_chan_config_t tx_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&tx_cfg, &tx_chan, NULL);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_new_channel TX failed: %s\n", esp_err_to_name(err));
        return false;
    }

    i2s_std_config_t tx_std = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SPK_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_16BIT,
                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SPK_BCK,
            .ws   = I2S_SPK_LRCK,
            .dout = I2S_SPK_DIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(tx_chan, &tx_std);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_channel_init_std_mode TX failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = i2s_channel_enable(tx_chan);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_channel_enable TX failed: %s\n", esp_err_to_name(err));
        return false;
    }

    Serial.println("OK: Speaker  (MAX98357A — I2S0, GPIO 26/25/27)");
    initialized = true;
    return true;
}

size_t AudioOutput::write(const int16_t* buffer, size_t numSamples) {
    if (!initialized || numSamples == 0) return 0;

    // Duplicate mono samples to stereo (L+R) for MAX98357A
    size_t chunk = min(numSamples, (size_t)SPK_BUFFER_SIZE);
    for (size_t i = 0; i < chunk; i++) {
        stereoBuf[i * 2]     = buffer[i];   // Left
        stereoBuf[i * 2 + 1] = buffer[i];   // Right
    }

    size_t bytesWritten = 0;
    i2s_channel_write(tx_chan, stereoBuf, chunk * 4, &bytesWritten, portMAX_DELAY);
    return bytesWritten / 4;
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
