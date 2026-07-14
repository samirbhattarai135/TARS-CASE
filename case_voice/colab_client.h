#ifndef COLAB_CLIENT_H
#define COLAB_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "audio_output.h"

class ColabClient {
public:
    ColabClient();
    bool begin(const char* ssid, const char* password);
    bool connect(const char* serverUrl);
    bool isConnected();
    void disconnect();

    // Send recorded audio to Personaplex and stream the reply straight to
    // the speaker as it downloads — replies can be 15+ s of audio, far more
    // than fits in RAM.
    // Returns the number of response samples played (0 on failure).
    size_t processAudio(const int16_t* audioData, size_t numSamples,
                        AudioOutput* audioOut);

private:
    bool initialized;
    bool connected;
    String serverUrl;
    String audioEndpoint;   // e.g. "https://server/api/audio"
};

#endif
