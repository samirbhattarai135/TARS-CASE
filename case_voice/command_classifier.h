#ifndef COMMAND_CLASSIFIER_H
#define COMMAND_CLASSIFIER_H

#include <Arduino.h>
#include "balance_control.h"

enum VoiceCommand {
    CMD_NONE,
    CMD_FORWARD,
    CMD_BACKWARD,
    CMD_STOP,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_COMPLEX  // Send to Colab
};

class CommandClassifier {
public:
    CommandClassifier();
    bool begin();
    VoiceCommand classify(int16_t* audioBuffer, size_t numSamples);
    MotorMode commandToMotorMode(VoiceCommand cmd);
    const char* commandToString(VoiceCommand cmd);

private:
    bool initialized;

    // Simple pattern matching (placeholder)
    // TODO: Replace with TFLite model in Phase 4
    bool matchesPattern(int16_t* buffer, size_t numSamples, const char* pattern);
};

#endif
