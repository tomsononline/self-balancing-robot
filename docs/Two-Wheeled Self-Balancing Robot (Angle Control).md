# **Self-Balancing Robot Documentation**

This document provides technical specifications, wiring guides, and a software overview for a self-balancing robot utilizing an Arduino-compatible microcontroller, an MPU6050 Inertial Measurement Unit (IMU), and an L298N motor driver. The control system relies on a Proportional-Integral-Derivative (PID) controller for stability.

## **1\. Hardware Requirements and Specifications**

The robot is designed to operate on a 12V power supply, which typically powers the motors through the L298N driver.

| Component | Function | Notes |
| :---- | :---- | :---- |
| **Microcontroller** | Arduino Nano / Uno | The brain; runs the PID control loop. |
| **IMU** | MPU6050 (Accelerometer/Gyro) | Provides the robot's angle (input for PID). Uses I2C communication. |
| **Motor Driver** | L298N H-Bridge | Controls the speed and direction of two DC motors. Requires PWM inputs. |
| **Power Supply** | 12V Battery/Adapter | Powers the L298N module (and potentially the Arduino via the driver's onboard regulator). |
| **Motors** | 2x DC Gear Motors | Geared motors suitable for 12V operation. |
| **Wheels** | 2x Wheels | Appropriate size for the motor shaft. |

## **2\. Wiring and Pin Mapping**

The provided sketch uses specific digital pins on the Arduino for communication and motor control.

### **MPU6050 (I2C Communication)**

| MPU6050 Pin | Arduino Pin | Function |
| :---- | :---- | :---- |
| VCC | 5V | Power |
| GND | GND | Ground |
| SCL | A5 | I2C Clock |
| SDA | A4 | I2C Data |
| **INT** | **D2** | **External Interrupt (Critical for DMP)** |

### **L298N Motor Driver**

The motor driver uses six digital pins for control, with the speed pins requiring **PWM (Pulse Width Modulation)** capability.

| L298N Pin | Arduino Pin | Sketch \#define | Function |
| :---- | :---- | :---- | :---- |
| **ENA** | **D5** (PWM) | ENA\_PWM | Left Motor Speed |
| **IN1** | D6 | IN1 | Left Motor Direction 1 |
| **IN2** | D7 | IN2 | Left Motor Direction 2 |
| **ENB** | **D9** (PWM) | ENB\_PWM | Right Motor Speed |
| **IN3** | D10 | IN3 | Right Motor Direction 1 |
| **IN4** | D11 | IN4 | Right Motor Direction 2 |

**Note:** The L298N is powered directly by the 12V supply. Ensure the driver's power jumper is correctly set to power the Arduino's 5V line, or power the Arduino separately.

## **3\. Software Overview and Control System**

The core of the robot's stability is the **PID Controller**, which attempts to maintain a setpoint (the perfect upright angle) by adjusting the motor speed (output) based on the current angle (input).

### **3.1. Required Libraries**

1. **Wire.h:** For I2C communication with the MPU6050.  
2. **I2Cdev.h** and **MPU6050\_6Axis\_MotionApps20.h:** The official I2Cdevlib libraries for the MPU6050, specifically utilizing the **DMP (Digital Motion Processor)** for highly accurate angle calculation.  
3. **PID\_v1.h:** The standard PID library used to calculate the necessary motor response.

### **3.2. Critical PID Tuning Parameters**

Tuning these four values in the MPUTrial.ino sketch is the most critical step for achieving stable balancing.

| Variable | Value (Current) | Description |
| :---- | :---- | :---- |
| setpoint | 180.1 | **The Target Angle.** This is the angle (in degrees, offset to be 0-360) where the robot is perfectly upright. This value **MUST** be calibrated for your specific robot. |
| Kp | 23 | **Proportional Gain.** Responds to the *current* error. Too high: fast corrections, but severe oscillations. |
| Ki | 140 | **Integral Gain.** Responds to the *accumulated* error over time (long-term drift). Corrects slow drift, but too high causes overshoot and sustained oscillation. |
| Kd | 0.9 | **Derivative Gain.** Responds to the *rate of change* of the error. Dampens oscillations and slows the robot down as it approaches the setpoint. |

### **3.3. Control Loop Breakdown**

The control system operates on an interrupt-driven model, which is highly efficient.

| Function/Code Section | Description |
| :---- | :---- |
| **dmpDataReady()** | This interrupt function runs every time the MPU6050 has new data ready, setting the mpuInterrupt flag. |
| **MPU Data Read** | In the loop(), new MPU data is read from the FIFO buffer only after the interrupt flag is set. The robot's angle is calculated from the quaternion and gravity vector using mpu.dmpGetYawPitchRoll(ypr, \&q, \&gravity);. |
| **Input Calculation** | input \= ypr\[2\] \* 180 / M\_PI \+ 180; calculates the angle. **It uses the Roll angle (ypr\[2\])**, which suggests the motors are mounted to compensate for side-to-side tilt (the axis the robot rolls on). The \+ 180 centers the value around the upright setpoint of 180 degrees. |
| **PID Compute** | pid.Compute(); is called continuously. It takes the difference between input (current angle) and setpoint (target angle) and generates the output value. |
| **setMotorSpeed()** | This function handles the PID output: **1\. Direction:** If output is positive, the motors drive "Forward" to push the top back up. If negative, they drive "Reverse" to catch the fall. **2\. Speed:** abs(output) is written to the L298N ENA/ENB pins using analogWrite() (PWM). |
| **Serial Output** | Serial.print(input); Serial.print(" \=\>"); Serial.println(output); provides critical debugging feedback, showing the current angle and the motor power being applied. |

