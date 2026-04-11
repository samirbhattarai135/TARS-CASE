#ifndef COLAB_CLIENT_H
#define COLAB_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>

class ColabClient {
public:
    ColabClient();
    bool begin(const char* ssid, const char* password);
    bool connect(const char* serverUrl);
    bool sendAudio(int16_t* audioBuffer, size_t numSamples);
    bool receiveAudio(int16_t* audioBuffer, size_t maxSamples, size_t* receivedSamples);
    bool isConnected();
    void disconnect();

private:
    bool initialized;
    bool connected;
    WiFiClient client;

    // TODO: Replace with WebSocket in Phase 6
};

#endif
