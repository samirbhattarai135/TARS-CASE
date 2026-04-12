/*
 * Audio Loopback: INMP441 Mic → ESP32 DAC → PAM8403 Speaker
 * CASE Self-Balancing Robot — Audio Subsystem
 *
 * ESP32 Arduino core 3.x / ESP-IDF 5.x compatible.
 * Uses new driver/i2s_std.h and driver/dac_continuous.h APIs.
 *
 * ─────────────────────────────────────────────────────────────
 * WIRING
 * ─────────────────────────────────────────────────────────────
 *
 *  INMP441 (Microphone)
 *  ──────────────────────────────────────
 *  VDD  →  3.3V
 *  GND  →  GND
 *  L/R  →  GND       (selects left channel)
 *  SCK  →  GPIO 32   (I2S bit clock)
 *  WS   →  GPIO 15   (I2S word select)
 *  SD   →  GPIO 4    (I2S data from mic)
 *
 *  PAM8403 (Amplifier)
 *  ──────────────────────────────────────
 *  5V   →  ESP32 VIN (5V rail)
 *  GND  →  GND
 *  L    →  GPIO 25   (DAC CH0 — analog audio)
 *  T    →  GND       (audio signal ground)
 *  R    →  GPIO 25   (tie to L → both speakers play mono)
 *
 *  Left  speaker → PAM8403 L output pair (L+ / L−)
 *  Right speaker → PAM8403 R output pair (R+ / R−)
 *
 * ─────────────────────────────────────────────────────────────
 * USAGE:
 *  1. Wire everything above
 *  2. Upload this sketch
 *  3. Open Serial Monitor at 115200 baud
 *  4. Speak into INMP441 → hear voice through PAM8403 speakers
 *  5. Adjust PAM8403 potentiometer for comfortable volume
 * ─────────────────────────────────────────────────────────────
 */

#include "driver/i2s_std.h"
#include "driver/dac_continuous.h"

// ── Pin definitions ────────────────────────────────────────
#define I2S_SCK     GPIO_NUM_32   // INMP441 SCK (bit clock)
#define I2S_WS      GPIO_NUM_15   // INMP441 WS  (word select)
#define I2S_SD      GPIO_NUM_4    // INMP441 SD  (data)
// DAC CH0 = GPIO 25 — set automatically by dac_continuous

// ── Audio parameters ───────────────────────────────────────
#define SAMPLE_RATE     16000     // Hz
#define READ_SAMPLES    512       // Samples per loop iteration
#define MIC_GAIN        1        // Amplify quiet INMP441 signal (1 = unity)

// ── Handles ────────────────────────────────────────────────
static i2s_chan_handle_t       rx_chan;
static dac_continuous_handle_t dac_handle;

// ── Buffers ────────────────────────────────────────────────
static int32_t micBuf[READ_SAMPLES];   // Raw 32-bit I2S samples from INMP441
static uint8_t dacBuf[READ_SAMPLES];   // 8-bit unsigned values for DAC (0–255)

// ── Setup ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println();
    Serial.println("╔══════════════════════════════════════════╗");
    Serial.println("║   INMP441 → PAM8403 Loopback Test       ║");
    Serial.println("║   ESP32-WROOM-32  |  Arduino core 3.x   ║");
    Serial.println("╚══════════════════════════════════════════╝");
    Serial.println();

    // ── 1. Create I2S RX channel ──────────────────────────
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_chan));

    // ── 2. Configure standard Philips I2S for INMP441 ─────
    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT,
                        I2S_SLOT_MODE_MONO),   // reads left slot only
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
    Serial.println("OK: Microphone   (INMP441 — GPIO 32 / 15 / 4)");

    // ── 3. Configure DAC continuous output on GPIO 25 ─────

    dac_continuous_config_t dac_cfg = {
        .chan_mask = DAC_CHANNEL_MASK_CH0,       // GPIO 25
        .desc_num  = 8,
        .buf_size  = 1024,
        .freq_hz   = SAMPLE_RATE,
        .offset    = 0,
        .clk_src   = DAC_DIGI_CLK_SRC_APLL,     // APLL: supports 648 Hz–MHz on ESP32
        .chan_mode  = DAC_CHANNEL_MODE_SIMUL,
    };
    ESP_ERROR_CHECK(dac_continuous_new_channels(&dac_cfg, &dac_handle));
    ESP_ERROR_CHECK(dac_continuous_enable(dac_handle));
    Serial.println("OK: Speaker      (PAM8403 — GPIO 25 via DAC)");

    Serial.println();
    Serial.println("Loopback active — speak into the INMP441.");
    Serial.println("Adjust PAM8403 potentiometer for volume.");
    Serial.println();
    Serial.println("Peak level (updates every second):");
    Serial.println("─────────────────────────────────────────");
}

// ── Main loop ──────────────────────────────────────────────
void loop() {
    size_t bytesRead    = 0;
    size_t bytesWritten = 0;

    // 1. Read a block of 32-bit I2S samples from INMP441
    esp_err_t err = i2s_channel_read(rx_chan, micBuf,
                                     READ_SAMPLES * sizeof(int32_t),
                                     &bytesRead, portMAX_DELAY);
    if (err != ESP_OK) return;

    int samples = (int)(bytesRead / sizeof(int32_t));

    // 2. Convert 32-bit I2S → 8-bit unsigned for DAC

    for (int i = 0; i < samples; i++) {
        int32_t s = (int32_t)micBuf[i] >> 24;
        s = constrain(s * MIC_GAIN, -128, 127);
        dacBuf[i] = (uint8_t)(s + 128);
    }

    // 3. Send to DAC — feeds GPIO 25 → PAM8403 L and R inputs
    dac_continuous_write(dac_handle, dacBuf, (size_t)samples, &bytesWritten, 1000);

    // 4. Serial status once per second
    static uint32_t lastStatus = 0;
    if (millis() - lastStatus >= 1000) {
        int32_t peak = 0;
        for (int i = 0; i < samples; i++) {
            int32_t s = abs(micBuf[i] >> 8);
            if (s > peak) peak = s;
        }
        int bar = (int)map(peak, 0, 2000000L, 0, 30);
        bar = constrain(bar, 0, 30);

        Serial.printf("Peak: %8ld  [", peak);
        for (int i = 0; i < 30; i++) Serial.print(i < bar ? '|' : '.');
        Serial.println(peak > 50000 ? "] ← sound" : "] (quiet)");

        lastStatus = millis();
    }
}
