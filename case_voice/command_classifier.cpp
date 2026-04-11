#include "command_classifier.h"

CommandClassifier::CommandClassifier() {
    initialized = false;
}

bool CommandClassifier::begin() {
    Serial.println("Command classifier initialized (placeholder)");
    Serial.println("TODO: Load TFLite model for command classification");
    initialized = true;
    return true;
}

VoiceCommand CommandClassifier::classify(int16_t* audioBuffer, size_t numSamples) {
    if (!initialized) return CMD_NONE;

    // PLACEHOLDER: Random classification for testing
    // TODO: Replace with TFLite model in Phase 4
    // This will be based on actual audio pattern matching

    // For now, return CMD_COMPLEX to trigger Colab processing
    return CMD_COMPLEX;
}

MotorMode CommandClassifier::commandToMotorMode(VoiceCommand cmd) {
    switch (cmd) {
        case CMD_FORWARD:
            return FORWARD_ASSIST;
        case CMD_BACKWARD:
            return BACKWARD_ASSIST;
        case CMD_TURN_LEFT:
            return TURN_LEFT;
        case CMD_TURN_RIGHT:
            return TURN_RIGHT;
        case CMD_STOP:
            return STOPPED;
        default:
            return BALANCE_ONLY;
    }
}

const char* CommandClassifier::commandToString(VoiceCommand cmd) {
    switch (cmd) {
        case CMD_FORWARD: return "FORWARD";
        case CMD_BACKWARD: return "BACKWARD";
        case CMD_STOP: return "STOP";
        case CMD_TURN_LEFT: return "TURN_LEFT";
        case CMD_TURN_RIGHT: return "TURN_RIGHT";
        case CMD_COMPLEX: return "COMPLEX";
        default: return "NONE";
    }
}
