/*
 * INMP441 I2S Audio Streaming — ESP32 → Laptop
 * CASE Self-Balancing Robot — Audio Subsystem
 *
 * ESP32 Arduino core 3.x / ESP-IDF 5.x compatible.
 * Uses new driver/i2s_std.h API.
 *
 * Streams 16 kHz / 16-bit / mono audio from INMP441
 * to laptop via USB serial. Use with play_audio.py.
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
 * PROTOCOL:
 *  Baud:    921600
 *  Format:  Binary — signed int16, little-endian
 *  Rate:    16 000 samples/sec
 *  Packet:  256 samples = 512 bytes per write()
 *
 * HANDSHAKE (called at startup AND on reconnect):
 *  1. ESP32 prints "AUDIO_STREAM_READY\n" every 500 ms
 *  2. Python sends 'G' → ESP32 starts streaming
 *
 * RECONNECT (no physical reset needed):
 *  Python sends 'R' while streaming → ESP32 re-runs handshake.
 *  play_audio.py always sends 'R' on connect so it works whether
 *  the ESP32 just booted or was already streaming from a prior run.
 *
 * USAGE:
 *  1. Upload this sketch
 *  2. CLOSE Arduino Serial Monitor (it locks the port)
 *  3. Run:  python3 play_audio.py
 */

#include "driver/i2s_std.h"

// ── Pins ───────────────────────────────────────────────────
#define I2S_SCK     GPIO_NUM_32
#define I2S_WS      GPIO_NUM_15
#define I2S_SD      GPIO_NUM_4

// ── Audio / serial parameters ──────────────────────────────
#define SAMPLE_RATE     16000   // Must match play_audio.py
#define BAUD_RATE       460800  // Must match play_audio.py — 115200 is too slow; 921600 unreliable on macOS CP2102
#define BUFFER_SAMPLES  256     // Samples per serial packet

static i2s_chan_handle_t rx_chan;
static int32_t i2sBuf[BUFFER_SAMPLES];
static int16_t outBuf[BUFFER_SAMPLES];

// ── Handshake ──────────────────────────────────────────────
void waitForGo() {
    while (Serial.available()) Serial.read();   // flush stale RX
    uint32_t lastAnnounce = 0;
    while (true) {
        if (millis() - lastAnnounce >= 500) {
            Serial.println("AUDIO_STREAM_READY");
            lastAnnounce = millis();
        }
        if (Serial.available()) {
            int b = Serial.read();
            if (b == 'G') break;   // GO — start / resume streaming
            // 'R' or anything else: keep announcing
        }
    }
    while (Serial.available()) Serial.read();   // consume leftover bytes
}

// ── Setup ──────────────────────────────────────────────────
void setup() {
    Serial.begin(BAUD_RATE);

    // ── Create RX channel ──────────────────────────────────
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    // ── Standard Philips I2S for INMP441 ──────────────────
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

    waitForGo();   // wait for Python to connect and send 'G'
}

// ── Main loop ──────────────────────────────────────────────
void loop() {
    // ── Reconnect support ──────────────────────────────────
    if (Serial.available()) {
        while (Serial.available()) Serial.read();   // drain RX
        waitForGo();
        return;
    }

    // ── Stream one packet ──────────────────────────────────
    size_t bytesRead = 0;
    esp_err_t err = i2s_channel_read(rx_chan, i2sBuf,
                                     BUFFER_SAMPLES * sizeof(int32_t),
                                     &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) return;

    int samples = (int)(bytesRead / sizeof(int32_t));

    for (int i = 0; i < samples; i++) {
        outBuf[i] = (int16_t)((int32_t)i2sBuf[i] >> 16);
    }

    // Send raw binary — play_audio.py reads BUFFER_SAMPLES * 2 bytes
    Serial.write((const uint8_t*)outBuf, samples * sizeof(int16_t));
}
