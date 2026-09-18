#ifndef I2S_BUS_H
#define I2S_BUS_H

#include <Arduino.h>
#include "driver/i2s_std.h"

// The board wires the INMP441 and the MAX98357A to the same bit clock and
// word select; only the data lines are separate. That makes the two directions
// one full-duplex I2S port rather than two independent ones, so the clock
// pins, sample rate and slot format are a single fact shared by the
// microphone and the amplifier, and they live here.
//
// Pin numbers follow the Self_balance board's net map in build_board.py.

#define I2S_BCLK        GPIO_NUM_18   // INMP441 SCK + MAX98357A BCLK
#define I2S_WS          GPIO_NUM_19   // INMP441 WS  + MAX98357A LRC
#define I2S_DIN         GPIO_NUM_34   // INMP441 SD, an input-only GPIO
#define I2S_DOUT        GPIO_NUM_23   // MAX98357A DIN

#define I2S_SAMPLE_RATE 16000

// Full duplex applies one clock and one slot format to both directions, so
// the microphone's native frame decides it: 32-bit slots, stereo. The
// INMP441 sends 24-bit samples left-aligned in a 32-bit slot and drives only
// the left one (its L/R pin is grounded). The MAX98357A accepts 32-bit frames
// and averages the two slots, so writing a sample to both reproduces it.
#define I2S_SLOT_BITS   I2S_DATA_BIT_WIDTH_32BIT

// Brings the shared port up on first call and does nothing on later ones, so
// AudioInput and AudioOutput can each call it during their own begin()
// without either having to go first.
bool i2sBusBegin();

// NULL until i2sBusBegin() has succeeded.
i2s_chan_handle_t i2sBusRx();
i2s_chan_handle_t i2sBusTx();

#endif
