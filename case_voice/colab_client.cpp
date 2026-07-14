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
                                  AudioOutput* audioOut) {
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
    http.setConnectTimeout(5000);
    // The bridge server pipeline (real-time paced upload to Moshi + 1 s
    // trailing silence + up to 15 s response collection) runs to completion
    // before the first response byte comes back.
    http.setTimeout(30000);

    size_t sendBytes = numSamples * sizeof(int16_t);
    Serial.printf("[Colab] Sending %d samples (%d bytes)...\n", numSamples, sendBytes);

    int httpCode = http.POST((uint8_t*)audioData, sendBytes);

    if (httpCode != 200) {
        Serial.printf("[Colab] Server returned HTTP %d\n", httpCode);
        http.end();
        return 0;
    }

    // Stream response audio (raw PCM int16 mono 16 kHz) straight to the
    // speaker. AudioOutput::write() blocks on the I2S DMA buffer, which
    // paces consumption to real time.
    int contentLen = http.getSize();  // -1 when chunked/unknown length
    WiFiClient* stream = http.getStreamPtr();

    static int16_t pcmBuf[256];
    uint8_t* raw = (uint8_t*)pcmBuf;
    size_t leftover = 0;        // odd byte carried between reads
    size_t totalBytes = 0;
    size_t samplesPlayed = 0;
    uint32_t lastDataMs = millis();

    while (stream->connected() || stream->available() > 0) {
        if (contentLen > 0 && totalBytes >= (size_t)contentLen) break;

        size_t avail = stream->available();
        if (avail == 0) {
            if (millis() - lastDataMs > 5000) {
                Serial.println("[Colab] Response stream stalled");
                break;
            }
            delay(1);
            continue;
        }
        lastDataMs = millis();

        size_t toRead = min(avail, sizeof(pcmBuf) - leftover);
        if (contentLen > 0) {
            toRead = min(toRead, (size_t)contentLen - totalBytes);
        }
        size_t got = stream->readBytes(raw + leftover, toRead);
        totalBytes += got;

        size_t haveBytes = leftover + got;
        size_t wholeSamples = haveBytes / sizeof(int16_t);
        if (wholeSamples > 0 && audioOut) {
            audioOut->write(pcmBuf, wholeSamples);
            samplesPlayed += wholeSamples;
        }
        leftover = haveBytes % sizeof(int16_t);
        if (leftover) raw[0] = raw[haveBytes - 1];
    }

    http.end();

    Serial.printf("[Colab] Played %d response samples (%.1f s)\n",
                  samplesPlayed, (float)samplesPlayed / SPK_SAMPLE_RATE);
    return samplesPlayed;
}

bool ColabClient::isConnected() {
    return connected && (WiFi.status() == WL_CONNECTED);
}

void ColabClient::disconnect() {
    connected = false;
    Serial.println("Disconnected from Personaplex server");
}
