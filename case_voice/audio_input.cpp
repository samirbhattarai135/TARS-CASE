#include "audio_input.h"
#include "driver/i2s_std.h"

// I2S channel handle — uses I2S_NUM_1 to keep I2S0 free for audio output
static i2s_chan_handle_t rx_chan = NULL;

// Temporary buffer for 32-bit I2S reads (INMP441 outputs 32-bit frames)
static int32_t i2sBuf[MIC_BUFFER_SIZE];

AudioInput::AudioInput() {
    initialized = false;
}

bool AudioInput::begin() {
    // Create I2S RX channel on I2S_NUM_1.
    // I2S0 is reserved for audio output (MAX98357A / PCM5102A / DAC).
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_chan);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_new_channel failed: %s\n", esp_err_to_name(err));
        return false;
    }

    // Standard Philips I2S config matching INMP441 requirements:
    // 32-bit slot width, mono (L/R pin tied to GND = left channel)
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT,
                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_MIC_SCK,
            .ws   = I2S_MIC_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_MIC_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_channel_init_std_mode failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = i2s_channel_enable(rx_chan);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_channel_enable failed: %s\n", esp_err_to_name(err));
        return false;
    }

    Serial.println("OK: Microphone (INMP441 — I2S1, GPIO 32/15/4)");
    initialized = true;
    return true;
}

size_t AudioInput::read(int16_t* buffer, size_t numSamples) {
    if (!initialized || numSamples == 0) return 0;

    // Clamp to internal buffer size
    if (numSamples > MIC_BUFFER_SIZE) numSamples = MIC_BUFFER_SIZE;

    size_t bytesRead = 0;
    esp_err_t err = i2s_channel_read(rx_chan, i2sBuf,
                                     numSamples * sizeof(int32_t),
                                     &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) return 0;

    size_t samples = bytesRead / sizeof(int32_t);

    // Convert 32-bit I2S → 16-bit signed.
    // INMP441: 24-bit audio left-aligned in 32-bit slot.
    // >> 16 preserves the sign bit and yields int16 range.
    for (size_t i = 0; i < samples; i++) {
        buffer[i] = (int16_t)((int32_t)i2sBuf[i] >> 16);
    }

    return samples;
}

bool AudioInput::isAvailable() {
    return initialized;
}
