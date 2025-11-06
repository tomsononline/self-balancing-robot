#include <Wire.h> // Required for I2C communication
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
MPU6050 mpu;

// --- L298N Motor Driver Pin Definitions ---
// ENA and ENB MUST be connected to Arduino PWM pins (e.g., D5, D6, D9, D10, D11 on Nano/Uno)
// INx pins are digital for direction control.

// Define Motor Pins based on standard L298N wiring
#define ENA_PWM 5   // Left Motor Speed (PWM Pin) - Adjust to your board's PWM pins
#define IN1 6       // Left Motor Direction 1 (Digital)
#define IN2 7       // Left Motor Direction 2 (Digital)

#define ENB_PWM 9  // Right Motor Speed (PWM Pin) - Adjust to your board's PWM pins
#define IN3 10      // Right Motor Direction 1 (Digital)
#define IN4 11      // Right Motor Direction 2 (Digital)
// IMPORTANT: Please verify these pins against your physical L298N wiring!


// MPU control/status vars
bool dmpReady = false;
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];
// orientation/motion vars
Quaternion q;           // [w, x, y, z] quaternion container
VectorFloat gravity;    // [x, y, z] gravity vector
float ypr[3];           // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

/*********Tune these 4 values for your BOT*********/
// *** CRITICAL ***: This is the angle where the robot is perfectly upright.
// Setpoint updated to 183.67
double setpoint = 180.1;
double Kp = 23;        // Proportional Gain (Start tuning this first)
double Kd = 0.9;       // Derivative Gain (Used to dampen oscillations)
double Ki = 140;       // Integral Gain (Used to correct long-term drift)
/******End of values setting*********/

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

// --- Motor Control Function Prototypes ---
void Forward();
void Reverse();
void Stop();
void setMotorSpeed(int speed);


// =========================================================================
// SETUP FUNCTION
// =========================================================================

void setup() {
  Wire.begin(); // Initialize I2C on A4/A5
  Serial.begin(115200);

  // --- MPU6050 DMP Setup ---
  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  // Verify connection
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();

  // supply your own gyro/accel offsets (The user explicitly requested NOT to change these)
  mpu.setXGyroOffset(55);
  mpu.setYGyroOffset(7);
  mpu.setZGyroOffset(2);
  mpu.setZAccelOffset(708);

  if (devStatus == 0)
  {
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    // Interrupt 0 is typically digital pin D2 on the Nano/Uno
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    // --- PID Setup ---
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10); // Update PID every 10ms
    pid.SetOutputLimits(-255, 255); // Motor PWM range
  }
  else
  {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // --- Motor Pin Initialization (L298N) ---
  pinMode(ENA_PWM, OUTPUT);
  pinMode(ENB_PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initial Stop
  Stop();
}

// =========================================================================
// MAIN LOOP
// =========================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // While the MPU hasn't generated an interrupt AND we don't have enough data:
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    // No new MPU data, but we can still run the PID loop based on the last reading
    pid.Compute();

    // Control speed based on PID output
    setMotorSpeed((int)output);

    // Print the value of Input and Output for debugging.
    Serial.print(input);
    Serial.print(" =>");
    Serial.println(output);
  }

  // --- Read New MPU Data ---
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  // check for overflow
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  }
  // check for DMP data ready interrupt
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Get quaternion, gravity, and Euler angles (ypr)
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Calculate the input angle for PID
    // CHANGED: Using Roll angle (ypr[2]) instead of Pitch angle (ypr[1])
    // Roll is typically the side-to-side wobble. This suggests the motors are mounted
    // to respond to a side-to-side imbalance instead of front-to-back.
    // We assume the offset needs to be maintained for the Roll axis.
    input = ypr[2] * 180 / M_PI + 180;
  }
}


// =========================================================================
// L298N Motor Control Functions
// =========================================================================

/**
   @brief Sets the speed and direction for both motors based on the signed PID output.
   @param motor_power Signed integer from -255 (full reverse) to 255 (full forward).
*/
void setMotorSpeed(int motor_power) {
  int speed = abs(motor_power);

  // Clamp speed to valid PWM range
  if (speed > 255) speed = 255;

  analogWrite(ENA_PWM, speed);
  analogWrite(ENB_PWM, speed);

  if (motor_power > 0) {
    // Robot falling in the positive direction -> Drive Forward/Right to catch
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    Serial.print("F");
  } else if (motor_power < 0) {
    // Robot falling in the negative direction -> Drive Backward/Left to recover
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    Serial.print("R");
  } else {
    // Balanced -> Stop (Brake)
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(ENA_PWM, 0);
    analogWrite(ENB_PWM, 0);
  }
}

// These functions are kept for structural completeness but their primary logic is now in setMotorSpeed()
void Forward()
{
  // Speed control is done in setMotorSpeed, these only set direction for potential use/debugging
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  Serial.print("F");
}

void Reverse()
{
  // Speed control is done in setMotorSpeed, these only set direction for potential use/debugging
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  Serial.print("R");
}

void Stop()
{
  // Stop motors
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA_PWM, 0);
  analogWrite(ENB_PWM, 0);
  Serial.print("S");
}
