/*
 * Audio Stream - Microphone to Laptop Speakers
 *
 * Streams audio from analog microphone to laptop via serial.
 * Use with companion Python script to hear audio on laptop speakers.
 *
 * WIRING:
 * VCC → 3.3V
 * GND → GND
 * AO  → GPIO 34
 */

#define MIC_PIN 34
#define SAMPLE_RATE 8000  // 8kHz (lower rate for serial bandwidth)
#define BUFFER_SIZE 128   // Small buffer for low latency

int16_t audioBuffer[BUFFER_SIZE];
int dcOffset = 2048;

void setup() {
  Serial.begin(115200);  // High baud rate for audio streaming

  pinMode(MIC_PIN, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  // Calibrate DC offset
  calibrateDCOffset();

  // Send header so Python knows we're ready
  Serial.println("AUDIO_STREAM_READY");
  delay(100);
}

void loop() {
  // Read audio samples
  for (int i = 0; i < BUFFER_SIZE; i++) {
    int rawValue = analogRead(MIC_PIN);

    // Remove DC offset and convert to signed 16-bit
    int centered = rawValue - dcOffset;
    audioBuffer[i] = centered * 16;  // Scale to 16-bit range

    // Timing for sample rate (8kHz = 125us per sample)
    delayMicroseconds(125);
  }

  // Send audio buffer as binary data
  Serial.write((uint8_t*)audioBuffer, BUFFER_SIZE * 2);
}

void calibrateDCOffset() {
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    sum += analogRead(MIC_PIN);
    delay(2);
  }
  dcOffset = sum / 500;
}
