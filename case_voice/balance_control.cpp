#include "balance_control.h"
#include <Arduino.h>
#include "Wire.h"

// Pin definitions
#define SDA_PIN 21
#define SCL_PIN 22
#define MPU_INT_PIN 2

#define PWMA 25
#define AIN1 26
#define AIN2 27
#define PWMB 14
#define BIN1 12
#define BIN2 13
#define STBY 33

// Global interrupt flag
volatile bool mpuInterrupt = false;

void IRAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

BalanceControl::BalanceControl() {
    setpoint = 182;  // Adjust based on calibration
    Kp = 15;
    Kd = 0.9;
    Ki = 140;

    input = 0;
    output = 0;
    dmpReady = false;
    currentMode = BALANCE_ONLY;

    pid = new PID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
}

bool BalanceControl::begin() {
    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);

    // Initialize motor pins
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    digitalWrite(STBY, HIGH);

    // Stop motors initially
    stop();

    // Initialize MPU6050
    Serial.println(F("Initializing MPU6050..."));
    mpu.initialize();

    if (!mpu.testConnection()) {
        Serial.println(F("MPU6050 connection failed!"));
        return false;
    }

    Serial.println(F("MPU6050 connection successful"));

    // Load DMP
    devStatus = mpu.dmpInitialize();

    // Gyro offsets (adjust based on calibration)
    mpu.setXGyroOffset(-479);
    mpu.setYGyroOffset(84);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(1638);

    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        pinMode(MPU_INT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready!"));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();

        // Setup PID
        pid->SetMode(AUTOMATIC);
        pid->SetSampleTime(10);
        pid->SetOutputLimits(-255, 255);

        return true;
    } else {
        Serial.print(F("DMP init failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        return false;
    }
}

void BalanceControl::update() {
    if (!dmpReady) return;

    // Check for MPU data
    if (mpuInterrupt && fifoCount >= packetSize) {
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
        fifoCount = mpu.getFIFOCount();

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow!"));
        } else if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

            mpu.getFIFOBytes(fifoBuffer, packetSize);
            fifoCount -= packetSize;

            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            input = ypr[1] * 180 / M_PI + 180;
        }
    }

    // Compute PID
    pid->Compute();

    // Apply motor control based on mode
    applyMotorControl();
}

void BalanceControl::applyMotorControl() {
    // Only apply control if robot is within reasonable angle range
    if (input > 150 && input < 200) {
        int basePWM = output;

        switch (currentMode) {
            case BALANCE_ONLY:
                if (output > 0) {
                    forward(abs(basePWM) / 2);
                } else if (output < 0) {
                    reverse(abs(basePWM) / 2);
                }
                break;

            case FORWARD_ASSIST:
                if (output > 0) {
                    forward(abs(basePWM) / 2 + 80);
                } else {
                    forward(80);
                }
                break;

            case BACKWARD_ASSIST:
                if (output < 0) {
                    reverse(abs(basePWM) / 2 + 80);
                } else {
                    reverse(80);
                }
                break;

            case TURN_LEFT:
                // Differential steering - left motor slower
                digitalWrite(AIN1, output > 0 ? HIGH : LOW);
                digitalWrite(AIN2, output > 0 ? LOW : HIGH);
                analogWrite(PWMA, abs(output) / 4);

                digitalWrite(BIN1, output > 0 ? HIGH : LOW);
                digitalWrite(BIN2, output > 0 ? LOW : HIGH);
                analogWrite(PWMB, abs(output) / 2);
                break;

            case TURN_RIGHT:
                // Differential steering - right motor slower
                digitalWrite(AIN1, output > 0 ? HIGH : LOW);
                digitalWrite(AIN2, output > 0 ? LOW : HIGH);
                analogWrite(PWMA, abs(output) / 2);

                digitalWrite(BIN1, output > 0 ? HIGH : LOW);
                digitalWrite(BIN2, output > 0 ? LOW : HIGH);
                analogWrite(PWMB, abs(output) / 4);
                break;

            case STOPPED:
                stop();
                break;
        }
    } else {
        stop();
    }
}

void BalanceControl::forward(int pwm) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, pwm);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, pwm);
}

void BalanceControl::reverse(int pwm) {
    pwm = constrain(pwm, 0, 255);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, pwm);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, pwm);
}

void BalanceControl::stop() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);
}

void BalanceControl::setMotorMode(MotorMode mode) {
    currentMode = mode;
}

MotorMode BalanceControl::getMotorMode() {
    return currentMode;
}

double BalanceControl::getAngle() {
    return input;
}

double BalanceControl::getPIDOutput() {
    return output;
}
