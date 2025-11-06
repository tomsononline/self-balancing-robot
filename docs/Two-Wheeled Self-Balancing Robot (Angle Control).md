# **Two-Wheeled Self-Balancing Robot (Angle Control)**

This project implements a classic two-wheeled inverted pendulum using an **Arduino Uno/Nano** microcontroller. It utilizes the **MPU6050's Digital Motion Processor (DMP)** for precise angle reading (sensor fusion) and a **PID (Proportional-Integral-Derivative) control loop** to maintain vertical balance.

This version focuses strictly on **Angle Control** and does not include motor encoder feedback (velocity/position control).

## **⚙️ Hardware Components**

* **Microcontroller:** Arduino Uno or Nano  
* **IMU Sensor:** MPU6050 (Accelerometer \+ Gyroscope, utilizing DMP)  
* **Motor Driver:** L298N Dual H-Bridge Module  
* **Motors:** 2 x 12V DC Geared Motors  
* **Power:** 12V Li-ion or Li-Po Battery Pack

## **🔌 Wiring Setup (L298N H-Bridge)**

This wiring setup is essential for connecting the **MPU6050** and the **L298N** motor driver to the Arduino.

| Component Pin | Arduino Pin / Connection | Notes |
| :---- | :---- | :---- |
| **MPU6050 SDA** | **A4** | I2C Data Line |
| **MPU6050 SCL** | **A5** | I2C Clock Line |
| **MPU6050 INT** | **D2** | External Interrupt 0 (Required for fast DMP data) |
| **L298N ENA (Left PWM)** | **D5** | Left Motor Speed Control (PWM) |
| **L298N ENB (Right PWM)** | **D10** | Right Motor Speed Control (PWM) |
| **L298N IN1, IN2, IN3, IN4** | **D6, D7, D12, D11** | Direction Control |
| **L298N 12V VMS** | **12V Battery Positive** | Motor Power Supply |
| **L298N GND** | **12V Battery Negative / Arduino GND** | Common Ground |

## **💻 Software & Libraries**

The sketch requires two essential libraries installed via the Arduino IDE's Library Manager:

1. **I2Cdevlib (by Jeff Rowberg):** Contains the necessary files (I2Cdev.h, MPU6050\_6Axis\_MotionApps20.h) to utilize the **MPU6050's DMP** for fused angle calculation.  
2. **Arduino PID Library (by Brett Beauregard):** Provides the robust PID\_v1.h control loop implementation.

## **🛠️ Calibration & Tuning**

The robot's stability relies entirely on tuning the PID loop and setting the correct angle **Setpoint**.

1. **Setpoint Determination:** Before enabling PID, determine the **precise angle** at which your robot stands perfectly vertical. The code currently uses 183.67, which should be adjusted after you measure your own stable reading using the Serial Monitor.  
2. **PID Tuning (Trial and Error):** Tune the constants sequentially:  
   * **Kp (Proportional \- Authority):** Start low and increase until the robot begins to oscillate (wobble quickly). This sets the basic responsiveness.  
   * **Kd (Derivative \- Damping):** Increase this value to aggressively reduce the fast oscillations caused by Kp, making the movement smoother.  
   * **Ki (Integral \- Drift Correction):** Increase this slowly to ensure the robot eliminates any long-term drift and holds the setpoint precisely.

Current starting values in self\_balancing\_bot.ino are: Kp=45, Kd=3.5, Ki=100.
