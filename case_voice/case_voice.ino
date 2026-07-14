/*
 * CASE Voice-Controlled Self-Balancing Robot
 * Integration of Nvidia Personaplex AI with ESP32 balance control
 *
 * Architecture:
 * - Core 1 (High Priority): Balance control loop (~100Hz, never blocked)
 * - Core 0 (Normal Priority): Audio processing + AI interaction
 *
 * Hardware:
 * - ESP32-WROOM-32
 * - MPU6050 IMU (I2C: GPIO 21/22)
 * - TB6612FNG Motor Driver
 * - INMP441 I2S Microphone (I2S1 RX: GPIO 32/15/4)
 * - MAX98357A I2S Amplifier (I2S0 TX: GPIO 26/25/27)
 *
 * Audio pipeline:
 *   INMP441 → I2S1 → ESP32 → WiFi → Personaplex AI (Colab)
 *   Personaplex AI → WiFi → ESP32 → I2S0 → MAX98357A → Speaker
 */

#include "balance_control.h"
#include "audio_input.h"
#include "audio_output.h"
#include "wake_word.h"
#include "command_classifier.h"
#include "colab_client.h"

// WiFi credentials (UPDATE THESE! — 2.4 GHz network only)
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";
// Cloudflare tunnel URL — points to personaplex_server.py proxy (port 9001),
// NOT directly to Moshi (port 8998). Update each time you restart the tunnel.
const char* COLAB_SERVER_URL = "https://your-tunnel-url.trycloudflare.com";

// FreeRTOS queue for motor commands
QueueHandle_t motorCommandQueue;

// Global objects
BalanceControl balanceControl;
AudioInput audioInput;
AudioOutput audioOutput;
WakeWordDetector wakeWordDetector;
CommandClassifier commandClassifier;
ColabClient colabClient;

// Audio buffers
// Small buffer for real-time mic reads (32ms chunks)
#define CHUNK_SAMPLES   512
static int16_t chunkBuf[CHUNK_SAMPLES];

// Voice command recording buffer (the AI reply is streamed straight to the
// speaker, so no response buffer is needed).
// 1.5s at 16 kHz = 24000 samples = 48 KB — fits in ESP32-WROOM-32's ~167 KB free heap.
#define RECORD_SAMPLES  24000
static int16_t* audioBuf = NULL;

// State machine
enum SystemState {
    STATE_IDLE,
    STATE_LISTENING,
    STATE_PROCESSING_LOCAL,
    STATE_PROCESSING_COLAB
};

volatile SystemState currentState = STATE_IDLE;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== CASE Voice-Controlled Robot ===");
    Serial.println("Initializing systems...\n");

    // Allocate single shared audio buffer from heap
    audioBuf = (int16_t*)malloc(RECORD_SAMPLES * sizeof(int16_t));
    if (!audioBuf) {
        Serial.println("ERROR: Failed to allocate audio buffer!");
        Serial.printf("  Requested: %d bytes\n", RECORD_SAMPLES * 2);
        Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());
        while (1) delay(1000);
    }
    Serial.printf("Audio buffer allocated (%d KB, free heap: %d KB)\n",
                  (RECORD_SAMPLES * 2) / 1024, ESP.getFreeHeap() / 1024);

    // Create motor command queue
    motorCommandQueue = xQueueCreate(10, sizeof(MotorMode));

    // Initialize balance control on Core 1 (high priority)
    if (!balanceControl.begin()) {
        Serial.println("ERROR: Balance control initialization failed!");
        while (1) delay(1000);
    }

    // Initialize audio systems
    Serial.println("\nInitializing audio systems...");

    if (!audioInput.begin()) {
        Serial.println("WARNING: Audio input initialization failed");
        Serial.println("Robot will work in balance-only mode");
    }

    if (!audioOutput.begin()) {
        Serial.println("WARNING: Audio output initialization failed");
    } else {
        // Play startup tone
        audioOutput.playTone(1000, 200);
        delay(100);
        audioOutput.playTone(1500, 200);
    }

    // Initialize AI components
    wakeWordDetector.begin();
    commandClassifier.begin();

    // Initialize WiFi and Colab client
    Serial.println("\nInitializing network...");
    if (colabClient.begin(WIFI_SSID, WIFI_PASSWORD)) {
        if (colabClient.connect(COLAB_SERVER_URL)) {
            Serial.println("Connected to Personaplex server!");
        } else {
            Serial.println("WARNING: Could not connect to Personaplex server");
            Serial.println("Robot will work in local-only mode");
        }
    }

    // FreeRTOS tasks
    Serial.println("\nStarting dual-core tasks...");

    xTaskCreatePinnedToCore(
        balanceTask,
        "BalanceTask",
        10000,
        NULL,
        2,  // High priority
        NULL,
        1   // Core 1
    );

    xTaskCreatePinnedToCore(
        audioTask,
        "AudioTask",
        20000,
        NULL,
        1,  // Normal priority
        NULL,
        0   // Core 0
    );

    Serial.println("\n=== System Ready ===");
    Serial.println("Say 'Hey CASE' to activate voice control");
    Serial.println("Balance control running on Core 1");
    Serial.println("Audio processing running on Core 0\n");
}

void loop() {
    // Empty — FreeRTOS tasks handle everything
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

// ========== CORE 1: BALANCE CONTROL TASK (HIGH PRIORITY) ==========
void balanceTask(void* parameter) {
    MotorMode voiceCommand;

    Serial.println("[Core 1] Balance task started");

    while (1) {
        // Update balance control (reads MPU6050, computes PID, controls motors)
        balanceControl.update();

        // Check for voice commands from audio task (non-blocking)
        if (xQueueReceive(motorCommandQueue, &voiceCommand, 0) == pdTRUE) {
            Serial.print("[Core 1] Received motor command: ");
            Serial.println(voiceCommand);
            balanceControl.setMotorMode(voiceCommand);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ========== CORE 0: AUDIO PROCESSING TASK (NORMAL PRIORITY) ==========
void audioTask(void* parameter) {
    Serial.println("[Core 0] Audio task started");

    while (1) {
        switch (currentState) {
            case STATE_IDLE: {
                // Listen for wake word using small chunks
                if (audioInput.isAvailable()) {
                    size_t samplesRead = audioInput.read(chunkBuf, CHUNK_SAMPLES);
                    if (samplesRead > 0 && wakeWordDetector.detect(chunkBuf, samplesRead)) {
                        Serial.println("\n[Audio] Wake word detected!");
                        audioOutput.playTone(2000, 100);  // Confirmation beep
                        currentState = STATE_LISTENING;
                    }
                }
                break;
            }

            case STATE_LISTENING: {
                // Record full voice command
                Serial.printf("[Audio] Recording %.1f seconds of audio...\n",
                              (float)RECORD_SAMPLES / MIC_SAMPLE_RATE);
                size_t totalRecorded = 0;

                while (totalRecorded < RECORD_SAMPLES) {
                    size_t remaining = RECORD_SAMPLES - totalRecorded;
                    size_t toRead = min(remaining, (size_t)CHUNK_SAMPLES);
                    size_t got = audioInput.read(audioBuf + totalRecorded, toRead);
                    totalRecorded += got;
                }

                Serial.printf("[Audio] Recorded %d samples\n", totalRecorded);
                audioOutput.playTone(1500, 50);  // End-of-recording beep

                // Classify the command locally first
                VoiceCommand cmd = commandClassifier.classify(audioBuf, totalRecorded);
                Serial.print("[Audio] Classified: ");
                Serial.println(commandClassifier.commandToString(cmd));

                if (cmd == CMD_COMPLEX) {
                    currentState = STATE_PROCESSING_COLAB;
                } else if (cmd != CMD_NONE) {
                    // Simple motor command — handle locally
                    MotorMode mode = commandClassifier.commandToMotorMode(cmd);
                    xQueueSend(motorCommandQueue, &mode, 0);
                    audioOutput.playTone(1500, 100);
                    currentState = STATE_IDLE;
                } else {
                    currentState = STATE_IDLE;
                }
                break;
            }

            case STATE_PROCESSING_COLAB: {
                Serial.println("[Audio] Sending to Personaplex AI...");

                if (colabClient.isConnected()) {
                    // Send recorded audio; the reply is streamed straight to
                    // the speaker as it downloads (no size limit)
                    size_t responseSamples = colabClient.processAudio(
                        audioBuf, RECORD_SAMPLES, &audioOutput
                    );

                    if (responseSamples == 0) {
                        Serial.println("[Audio] No response from Personaplex");
                        audioOutput.playTone(500, 300);  // Error tone
                    }
                } else {
                    Serial.println("[Audio] Personaplex not connected");
                    audioOutput.playTone(500, 300);
                }

                currentState = STATE_IDLE;
                break;
            }

            default:
                currentState = STATE_IDLE;
                break;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
