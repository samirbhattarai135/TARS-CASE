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

bool ColabClient::connect(const char* serverUrl) {
    Serial.println("Colab client connect (placeholder)");
    Serial.println("TODO: Implement WebSocket connection in Phase 6");
    Serial.print("Server URL: ");
    Serial.println(serverUrl);

    // Placeholder - pretend we're connected
    connected = false;  // Set to false until actual implementation
    return false;
}

bool ColabClient::sendAudio(int16_t* audioBuffer, size_t numSamples) {
    if (!connected) {
        Serial.println("Cannot send audio - not connected to Colab");
        return false;
    }

    // TODO: Implement WebSocket audio streaming in Phase 6
    Serial.printf("TODO: Send %d audio samples to Colab\n", numSamples);
    return false;
}

bool ColabClient::receiveAudio(int16_t* audioBuffer, size_t maxSamples, size_t* receivedSamples) {
    if (!connected) {
        Serial.println("Cannot receive audio - not connected to Colab");
        return false;
    }

    // TODO: Implement WebSocket audio reception in Phase 6
    Serial.println("TODO: Receive audio response from Colab");
    *receivedSamples = 0;
    return false;
}

bool ColabClient::isConnected() {
    return connected && (WiFi.status() == WL_CONNECTED);
}

void ColabClient::disconnect() {
    connected = false;
    Serial.println("Disconnected from Colab server");
}
