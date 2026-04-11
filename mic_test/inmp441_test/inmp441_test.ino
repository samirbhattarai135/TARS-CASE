/*
 * INMP441 I2S Microphone Verification Test
 * CASE Self-Balancing Robot — Audio Subsystem
 *
 * ESP32 Arduino core 3.x / ESP-IDF 5.x compatible.
 * Uses new driver/i2s_std.h API.
 *
 * ─────────────────────────────────────────────────────────────
 * WIRING — INMP441 → ESP32
 * ─────────────────────────────────────────────────────────────
 *  VDD  →  3.3V
 *  GND  →  GND
 *  L/R  →  GND       (left channel)
 *  SCK  →  GPIO 32   (bit clock)
 *  WS   →  GPIO 15   (word select)
 *  SD   →  GPIO 4    (data from mic)
 * ─────────────────────────────────────────────────────────────
 *
 * USAGE:
 *  1. Upload this sketch
 *  2. Open Serial Monitor at 115200 baud
 *  3. Speak or clap — watch the level bar respond
 *
 *  For a live waveform: Tools → Serial Plotter
 *  (uncomment the Serial Plotter block at the bottom of loop())
 */

#include "driver/i2s_std.h"

// ── Pins ───────────────────────────────────────────────────
#define I2S_SCK     GPIO_NUM_32
#define I2S_WS      GPIO_NUM_15
#define I2S_SD      GPIO_NUM_4

// ── Audio parameters ───────────────────────────────────────
#define SAMPLE_RATE     16000
#define READ_SAMPLES    512

static i2s_chan_handle_t rx_chan;
static int32_t i2sBuf[READ_SAMPLES];

// ── Setup ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   INMP441 I2S Microphone Test — CASE    ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();
    Serial.println("Pin config:");
    Serial.println("  INMP441 VDD → 3.3V");
    Serial.println("  INMP441 GND → GND");
    Serial.println("  INMP441 L/R → GND   (left channel)");
    Serial.println("  INMP441 SCK → GPIO 32");
    Serial.println("  INMP441 WS  → GPIO 15");
    Serial.println("  INMP441 SD  → GPIO 4");
    Serial.println();

    // ── Create RX channel ──────────────────────────────────
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    // ── Standard Philips I2S config for INMP441 ───────────
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT,
                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_SCK,
            .ws   = I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

    Serial.println("OK: INMP441 I2S driver ready");
    Serial.println();
    Serial.println("Speak or clap near the microphone...");
    Serial.println("─────────────────────────────────────────");
    Serial.println();
    delay(300);
}

// ── Main loop ──────────────────────────────────────────────
void loop() {
    size_t bytesRead = 0;

    esp_err_t err = i2s_channel_read(rx_chan, i2sBuf,
                                     READ_SAMPLES * sizeof(int32_t),
                                     &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) return;

    int samples = (int)(bytesRead / sizeof(int32_t));

    // INMP441: 24-bit audio in top 24 bits of int32.
    // Shift right 8 → signed 24-bit for level measurement.
    int64_t sumAbs = 0;
    int32_t peak   = 0;
    for (int i = 0; i < samples; i++) {
        int32_t s = (int32_t)i2sBuf[i] >> 8;
        int32_t a = abs(s);
        sumAbs += a;
        if (a > peak) peak = a;
    }
    int32_t avg = (int32_t)(sumAbs / samples);

    // ── Serial Monitor output (every 150 ms) ──────────────
    static uint32_t lastPrint = 0;
    if (millis() - lastPrint >= 150) {
        int bar = (int)map(peak, 0, 2000000L, 0, 40);
        bar = constrain(bar, 0, 40);

        Serial.printf("Avg: %8ld  Peak: %8ld  [", avg, peak);
        for (int i = 0; i < 40; i++) Serial.print(i < bar ? '|' : '.');
        Serial.println(']');

        lastPrint = millis();
    }

    // ── Serial Plotter output (uncomment to use) ──────────
    // Comment out the Serial Monitor block above first, then
    // uncomment these two lines:
    //
    // Serial.printf("Avg:%ld,Peak:%ld\n",
    //               avg / 1000, peak / 1000);
}
