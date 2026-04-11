/*
 * CASE Voice-Controlled Self-Balancing Robot
 * Integration of Nvidia Personaplex AI with ESP32 balance control
 *
 * Architecture:
 * - Core 1 (High Priority): Balance control loop (~100Hz, never blocked)
 * - Core 0 (Normal Priority): Audio processing + AI interaction
 *
 * Hardware:
 * - ESP32-S3 or ESP32-WROVER with PSRAM
 * - MPU6050 IMU (I2C)
 * - TB6612FNG Motor Driver
 * - INMP441 I2S Microphone
 * - MAX98357A I2S Amplifier + Speaker
 */

#include "balance_control.h"
#include "audio_input.h"
#include "audio_output.h"
#include "wake_word.h"
#include "command_classifier.h"
#include "colab_client.h"

// WiFi credentials (UPDATE THESE!)
const char* WIFI_SSID = "we";
const char* WIFI_PASSWORD = "wewewewe!1";
const char* COLAB_SERVER_URL = "https://goal-pot-groundwater-providers.trycloudflare.com/";

// FreeRTOS queue for motor commands
QueueHandle_t motorCommandQueue;

// Global objects
BalanceControl balanceControl;i
AudioInput audioInput;
AudioOutput audioOutput;
WakeWordDetector wakeWordDetector;
CommandClassifier commandClassifier;
ColabClient colabClient;

// Audio buffer
#define AUDIO_BUFFER_SIZE 512
int16_t audioBuffer[AUDIO_BUFFER_SIZE];

// State machine
enum SystemState {
    STATE_IDLE,
    STATE_LISTENING,
    STATE_PROCESSING_LOCAL,
    STATE_PROCESSING_COLAB,
    STATE_SPEAKING
};

SystemState currentState = STATE_IDLE;

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n=== CASE Voice-Controlled Robot ===");
    Serial.println("Initializing systems...\n");

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
            Serial.println("Connected to Colab Personaplex server!");
        } else {
            Serial.println("WARNING: Could not connect to Colab server");
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
    // Empty - FreeRTOS tasks handle everything
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

            // Auto-return to balance mode after 3 seconds for movement commands
            if (voiceCommand != BALANCE_ONLY && voiceCommand != STOPPED) {
                // TODO: Add timeout mechanism
            }
        }

        // Small delay to maintain ~100Hz loop
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// ========== CORE 0: AUDIO PROCESSING TASK (NORMAL PRIORITY) ==========
void audioTask(void* parameter) {
    Serial.println("[Core 0] Audio task started");

    while (1) {
        switch (currentState) {
            case STATE_IDLE:
                // Listen for wake word
                if (audioInput.isAvailable()) {
                    size_t samplesRead = audioInput.read(audioBuffer, AUDIO_BUFFER_SIZE);

                    if (samplesRead > 0) {
                        if (wakeWordDetector.detect(audioBuffer, samplesRead)) {
                            Serial.println("\n[Audio] Wake word detected!");
                            audioOutput.playTone(2000, 100);  // Confirmation beep
                            currentState = STATE_LISTENING;
                        }
                    }
                }
                break;

            case STATE_LISTENING:
                // Record audio for command
                Serial.println("[Audio] Listening for command...");

                // TODO: Record 2-3 seconds of audio
                size_t samplesRead = audioInput.read(audioBuffer, AUDIO_BUFFER_SIZE);

                if (samplesRead > 0) {
                    // Classify command
                    VoiceCommand cmd = commandClassifier.classify(audioBuffer, samplesRead);

                    Serial.print("[Audio] Classified command: ");
                    Serial.println(commandClassifier.commandToString(cmd));

                    if (cmd == CMD_COMPLEX) {
                        currentState = STATE_PROCESSING_COLAB;
                    } else if (cmd != CMD_NONE) {
                        currentState = STATE_PROCESSING_LOCAL;

                        // Send motor command to balance task
                        MotorMode mode = commandClassifier.commandToMotorMode(cmd);
                        xQueueSend(motorCommandQueue, &mode, 0);

                        audioOutput.playTone(1500, 100);  // Acknowledgment
                        currentState = STATE_IDLE;
                    } else {
                        currentState = STATE_IDLE;
                    }
                }
                break;

            case STATE_PROCESSING_COLAB:
                Serial.println("[Audio] Processing with Colab...");

                if (colabClient.isConnected()) {
                    // Send audio to Colab Personaplex
                    if (colabClient.sendAudio(audioBuffer, AUDIO_BUFFER_SIZE)) {
                        // Wait for response
                        size_t receivedSamples;
                        if (colabClient.receiveAudio(audioBuffer, AUDIO_BUFFER_SIZE, &receivedSamples)) {
                            currentState = STATE_SPEAKING;
                        } else {
                            Serial.println("[Audio] Failed to receive response");
                            audioOutput.playTone(500, 300);  // Error tone
                            currentState = STATE_IDLE;
                        }
                    } else {
                        Serial.println("[Audio] Failed to send audio");
                        audioOutput.playTone(500, 300);
                        currentState = STATE_IDLE;
                    }
                } else {
                    Serial.println("[Audio] Colab not connected - I'm offline");
                    // TODO: Play "I'm offline" audio message
                    audioOutput.playTone(500, 300);
                    currentState = STATE_IDLE;
                }
                break;

            case STATE_SPEAKING:
                Serial.println("[Audio] Playing response...");

                // Play received audio from Personaplex
                audioOutput.write(audioBuffer, AUDIO_BUFFER_SIZE);

                currentState = STATE_IDLE;
                break;
        }

        // Small delay
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
