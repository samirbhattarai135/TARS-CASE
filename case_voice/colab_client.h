#ifndef COLAB_CLIENT_H
#define COLAB_CLIENT_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>

// Maximum recording: 1.5s at 16 kHz = 24000 samples = 48 KB
// ESP32-WROOM-32 has ~167 KB free heap after boot
#define MAX_RECORD_SAMPLES  24000
#define MAX_RECORD_BYTES    (MAX_RECORD_SAMPLES * sizeof(int16_t))

class ColabClient {
public:
    ColabClient();
    bool begin(const char* ssid, const char* password);
    bool connect(const char* serverUrl);
    bool isConnected();
    void disconnect();

    // Record audio from mic, send to Personaplex, receive response audio.
    // responseBuffer must be at least maxResponseSamples * 2 bytes.
    // Returns the number of response audio samples written to responseBuffer.
    // Returns 0 on failure.
    size_t processAudio(const int16_t* audioData, size_t numSamples,
                        int16_t* responseBuffer, size_t maxResponseSamples);

    // Legacy interface (for backward compatibility with case_voice.ino)
    bool sendAudio(int16_t* audioBuffer, size_t numSamples);
    bool receiveAudio(int16_t* audioBuffer, size_t maxSamples, size_t* receivedSamples);

private:
    bool initialized;
    bool connected;
    String serverUrl;
    String audioEndpoint;   // e.g. "https://server/api/audio"
};

#endif
