Two-Wheeled Self-Balancing Robot (Angle Control)

This project implements a classic two-wheeled inverted pendulum using an Arduino Uno/Nano microcontroller. It utilizes the MPU6050's Digital Motion Processor (DMP) for precise angle reading (sensor fusion) and a PID (Proportional-Integral-Derivative) control loop to maintain vertical balance.

This version focuses strictly on Angle Control and does not include motor encoder feedback (velocity/position control).

⚙️ Hardware Components

Microcontroller: Arduino Uno or Nano

IMU Sensor: M MPU6050 (Accelerometer + Gyroscope, utilizing DMP)

Motor Driver: L298N Dual H-Bridge Module

Motors: 2 x 12V DC Geared Motors

Power: 12V Li-ion or Li-Po Battery Pack

🔌 Wiring Setup

Component Pin

Arduino Pin / Connection

Notes

MPU6050 SDA

A4

I2C Data Line

MPU6050 SCL

A5

I2C Clock Line

MPU6050 INT

D2

External Interrupt 0 (Required for fast DMP reading)

L298N ENA (Left PWM)

D5

Left Motor Speed Control

L298N ENB (Right PWM)

D10

Right Motor Speed Control

L298N IN1, IN2, IN3, IN4

D6, D7, D12, D11

Direction Control

L298N 12V VMS

12V Battery Positive

Motor Power Supply

L298N GND

12V Battery Negative / Arduino GND

Common Ground

💻 Software & Libraries

This sketch requires the following libraries to be installed in the Arduino IDE:

I2Cdevlib (by Jeff Rowberg): Contains the necessary files (I2Cdev.h, MPU6050_6Axis_MotionApps20.h) for communicating with and utilizing the MPU6050's DMP.

Arduino PID Library (by Brett Beauregard): Provides the robust PID_v1.h control loop implementation.

🛠️ Calibration & Tuning

The success of this robot depends entirely on the correct setup of these parameters:

Offsets: The mpu.setXGyroOffset(...) values in setup() must be specific to your sensor's drift/bias.

Setpoint: The setpoint variable (183.67 currently) is the measured angle when the robot is perfectly vertical.

PID Tuning: The current Kp=45, Kd=3.5, Ki=100 are aggressive starting points. Tune them sequentially:

Kp: Increase until oscillation starts.

Kd: Increase to dampen and minimize oscillation.

Ki: Fine-tune to eliminate slow drift.
