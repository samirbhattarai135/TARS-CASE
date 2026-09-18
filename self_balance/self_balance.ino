#include "I2Cdev.h"
#include "Wire.h"
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;

// ESP32 I2C pins
#define SDA_PIN 21
#define SCL_PIN 22

// MPU6050 interrupt pin. Input-only GPIO, which is all an interrupt needs,
// and it keeps the IMU off the IO2 strapping pin.
#define MPU_INT_PIN 35

// TB6612FNG Motor Driver pins. These follow the Self_balance board, whose net
// map in build_board.py is the authoritative record; the same numbers are
// duplicated in case_voice/balance_control.cpp because Arduino keeps the two
// sketches separate.
#define PWMA 25      // Motor A speed (PWM)
#define AIN1 26      // Motor A direction
#define AIN2 27      // Motor A direction
#define PWMB 14      // Motor B speed (PWM)
#define BIN1 13      // Motor B direction
#define BIN2 32      // Motor B direction — IO32, not the IO12 strapping pin
#define STBY 33      // Standby pin - must be HIGH to enable driver

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/*********Tune these 4 values for your BOT*********/
double setpoint= 182; //set the value when the bot is perpendicular to ground using serial monitor. 


//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 15; //21 Set this first
double Kd = 0.9; //0.8 Set this secound
double Ki = 140; //140 Finally set this 


/******End of values setting*********/
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


void IRAM_ATTR dmpDataReady()
{
    mpuInterrupt = true;
}

void scanI2C() {
  Serial.println(F("Scanning I2C bus..."));
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("Found device at 0x"));
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  if (count == 0) {
    Serial.println(F("No I2C devices found! Check wiring:"));
    Serial.println(F("  - SDA -> GPIO 21"));
    Serial.println(F("  - SCL -> GPIO 22"));
    Serial.println(F("  - VCC -> 3.3V (NOT 5V!)"));
    Serial.println(F("  - GND -> GND"));
  } else {
    Serial.print(F("Found "));
    Serial.print(count);
    Serial.println(F(" device(s)"));
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Give serial time to connect

  // Initialize I2C with ESP32 pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000); // 400kHz I2C clock

  // Scan for I2C devices first
  scanI2C();

  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

     // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-479);
    mpu.setYGyroOffset(84);
    mpu.setZGyroOffset(15);
    mpu.setZAccelOffset(1638); 

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable ESP32 interrupt detection
        Serial.println(F("Enabling interrupt detection..."));
        pinMode(MPU_INT_PIN, INPUT);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();

        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

//Initialise TB6612FNG Motor Driver pins
    pinMode(PWMA, OUTPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(STBY, OUTPUT);

    // Enable the motor driver
    digitalWrite(STBY, HIGH);

    // Stop motors initially
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}


void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid.Compute();

        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input); Serial.print(" =>"); Serial.println(output);

        if (input>150 && input<200){//If the Bot is falling 
        if (output>0) //Falling towards front 
        Forward(); //Rotate the wheels forward 
        else if (output<0) //Falling towards back
        Reverse(); //Rotate the wheels backward 
        }
        else //If Bot not falling
        Stop(); //Hold the wheels still
    }


    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)

    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
        input = ypr[1] * 180/M_PI + 180;
   }
}


void Forward() //Code to rotate the wheel forward
{
    // Motor A forward
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, output / 2);

    // Motor B forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, output / 2);

    Serial.print("F"); //Debugging information
}


void Reverse() //Code to rotate the wheel Backward
{
    // Motor A reverse
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -output / 2); // output is negative, so negate it

    // Motor B reverse
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -output / 2);

    Serial.print("R");
}


void Stop() //Code to stop both the wheels
{
    // Brake mode (both inputs HIGH or LOW)
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, 0);

    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, 0);

    Serial.print("S");
}