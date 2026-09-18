#include "i2s_bus.h"

static i2s_chan_handle_t rx_chan = NULL;
static i2s_chan_handle_t tx_chan = NULL;
static bool busReady = false;

i2s_chan_handle_t i2sBusRx() { return rx_chan; }
i2s_chan_handle_t i2sBusTx() { return tx_chan; }

bool i2sBusBegin() {
    if (busReady) return true;

    // Both handles come from one call, which is what puts the two directions
    // on the same controller and lets them share the clock lines.
    i2s_chan_config_t chan_cfg =
        I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, &tx_chan, &rx_chan);
    if (err != ESP_OK) {
        Serial.printf("ERROR: i2s_new_channel failed: %s\n",
                      esp_err_to_name(err));
        return false;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(I2S_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_SLOT_BITS, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK,
            .ws   = I2S_WS,
            .dout = I2S_DOUT,
            .din  = I2S_DIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    // Both channels take the same configuration. In full duplex the driver
    // requires it -- one of them owns the clock and the other follows it.
    err = i2s_channel_init_std_mode(tx_chan, &std_cfg);
    if (err != ESP_OK) {
        Serial.printf("ERROR: I2S TX init failed: %s\n", esp_err_to_name(err));
        return false;
    }
    err = i2s_channel_init_std_mode(rx_chan, &std_cfg);
    if (err != ESP_OK) {
        Serial.printf("ERROR: I2S RX init failed: %s\n", esp_err_to_name(err));
        return false;
    }

    err = i2s_channel_enable(tx_chan);
    if (err != ESP_OK) {
        Serial.printf("ERROR: I2S TX enable failed: %s\n",
                      esp_err_to_name(err));
        return false;
    }
    err = i2s_channel_enable(rx_chan);
    if (err != ESP_OK) {
        Serial.printf("ERROR: I2S RX enable failed: %s\n",
                      esp_err_to_name(err));
        return false;
    }

    Serial.printf("OK: I2S full duplex on BCLK %d / WS %d, in %d, out %d\n",
                  I2S_BCLK, I2S_WS, I2S_DIN, I2S_DOUT);
    busReady = true;
    return true;
}
