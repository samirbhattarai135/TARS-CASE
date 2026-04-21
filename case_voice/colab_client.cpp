#include "colab_client.h"

ColabClient::ColabClient() {
    initialized = false;
    connected = false;
}

bool ColabClient::begin(const char* ssid, const char* password) {
    Serial.print("Connecting to WiFi: ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi connected!");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        initialized = true;
        return true;
    } else {
        Serial.println("\nWiFi connection failed!");
        return false;
    }
}

bool ColabClient::connect(const char* url) {
    if (!initialized) return false;

    serverUrl = String(url);
    // Ensure no trailing slash
    if (serverUrl.endsWith("/")) {
        serverUrl = serverUrl.substring(0, serverUrl.length() - 1);
    }
    audioEndpoint = serverUrl + "/api/audio";

    // Test connectivity with a simple GET
    HTTPClient http;
    http.begin(serverUrl + "/health");
    http.setTimeout(5000);
    int code = http.GET();
    http.end();

    if (code == 200) {
        Serial.println("Connected to Personaplex server!");
        Serial.print("  Audio endpoint: ");
        Serial.println(audioEndpoint);
        connected = true;
        return true;
    } else {
        Serial.printf("Server health check failed (HTTP %d)\n", code);
        Serial.println("Robot will work in local-only mode");
        // Still allow future retries
        connected = false;
        return false;
    }
}

size_t ColabClient::processAudio(const int16_t* audioData, size_t numSamples,
                                  int16_t* responseBuffer, size_t maxResponseSamples) {
    if (!connected || !initialized) return 0;

    // Re-check WiFi
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[Colab] WiFi disconnected");
        connected = false;
        return 0;
    }

    HTTPClient http;
    http.begin(audioEndpoint);
    http.addHeader("Content-Type", "application/octet-stream");
    http.addHeader("X-Sample-Rate", "16000");
    http.addHeader("X-Bit-Depth", "16");
    http.addHeader("X-Channels", "1");
    http.setTimeout(10000);  // 10 s for AI processing

    size_t sendBytes = numSamples * sizeof(int16_t);
    Serial.printf("[Colab] Sending %d samples (%d bytes)...\n", numSamples, sendBytes);

    int httpCode = http.POST((uint8_t*)audioData, sendBytes);

    if (httpCode != 200) {
        Serial.printf("[Colab] Server returned HTTP %d\n", httpCode);
        http.end();
        return 0;
    }

    // Read response audio (raw PCM int16 mono 16 kHz)
    int responseLen = http.getSize();
    if (responseLen <= 0) {
        Serial.println("[Colab] Empty response");
        http.end();
        return 0;
    }

    size_t responseSamples = responseLen / sizeof(int16_t);
    if (responseSamples > maxResponseSamples) {
        responseSamples = maxResponseSamples;
    }

    WiFiClient* stream = http.getStreamPtr();
    size_t bytesRead = 0;
    size_t totalBytes = responseSamples * sizeof(int16_t);
    uint8_t* dst = (uint8_t*)responseBuffer;

    while (bytesRead < totalBytes && stream->connected()) {
        size_t avail = stream->available();
        if (avail == 0) {
            delay(1);
            continue;
        }
        size_t toRead = min(avail, totalBytes - bytesRead);
        size_t got = stream->readBytes(dst + bytesRead, toRead);
        bytesRead += got;
    }

    http.end();

    responseSamples = bytesRead / sizeof(int16_t);
    Serial.printf("[Colab] Received %d response samples\n", responseSamples);
    return responseSamples;
}

// Legacy interface wrappers for case_voice.ino compatibility
static int16_t* pendingResponse = NULL;
static size_t pendingResponseSamples = 0;

bool ColabClient::sendAudio(int16_t* audioBuffer, size_t numSamples) {
    // Allocate response buffer if needed
    if (!pendingResponse) {
        pendingResponse = (int16_t*)malloc(MAX_RECORD_BYTES);
        if (!pendingResponse) {
            Serial.println("[Colab] Failed to allocate response buffer");
            return false;
        }
    }

    pendingResponseSamples = processAudio(audioBuffer, numSamples,
                                           pendingResponse, MAX_RECORD_SAMPLES);
    return (pendingResponseSamples > 0);
}

bool ColabClient::receiveAudio(int16_t* audioBuffer, size_t maxSamples, size_t* receivedSamples) {
    if (pendingResponseSamples == 0 || !pendingResponse) {
        *receivedSamples = 0;
        return false;
    }

    size_t toCopy = min(pendingResponseSamples, maxSamples);
    memcpy(audioBuffer, pendingResponse, toCopy * sizeof(int16_t));
    *receivedSamples = toCopy;
    pendingResponseSamples = 0;
    return true;
}

bool ColabClient::isConnected() {
    return connected && (WiFi.status() == WL_CONNECTED);
}

void ColabClient::disconnect() {
    connected = false;
    if (pendingResponse) {
        free(pendingResponse);
        pendingResponse = NULL;
    }
    Serial.println("Disconnected from Personaplex server");
}
