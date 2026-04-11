#ifndef BALANCE_CONTROL_H
#define BALANCE_CONTROL_H

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>

// Motor modes for voice command integration
enum MotorMode {
    BALANCE_ONLY,
    FORWARD_ASSIST,
    BACKWARD_ASSIST,
    TURN_LEFT,
    TURN_RIGHT,
    STOPPED
};

class BalanceControl {
public:
    BalanceControl();
    bool begin();
    void update();
    void setMotorMode(MotorMode mode);
    MotorMode getMotorMode();
    double getAngle();
    double getPIDOutput();

private:
    void initMPU6050();
    void initMotors();
    void readMPU6050();
    void applyMotorControl();
    void forward(int pwm);
    void reverse(int pwm);
    void stop();

    MPU6050 mpu;
    PID* pid;

    // PID tuning
    double setpoint;
    double input;
    double output;
    double Kp, Ki, Kd;

    // MPU6050 variables
    bool dmpReady;
    uint8_t mpuIntStatus;
    uint8_t devStatus;
    uint16_t packetSize;
    uint16_t fifoCount;
    uint8_t fifoBuffer[64];

    Quaternion q;
    VectorFloat gravity;
    float ypr[3];

    volatile bool* mpuInterruptPtr;
    MotorMode currentMode;
};

// Interrupt handler (must be external)
void IRAM_ATTR dmpDataReady();

#endif
