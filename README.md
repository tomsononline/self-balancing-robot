# **Two-Wheeled Self-Balancing Robot**

A real-time control project utilizing an Arduino microcontroller and a PID control loop to achieve dynamic single-axis (Roll) balancing.

## **1\. Project Overview**

This project implements a classic two-wheeled inverted pendulum. Stability is achieved by continuously reading the robot's tilt angle using the **MPU6050's Digital Motion Processor (DMP)** and applying corrective motor speed based on the error calculated by a **PID control loop**.

* **Control Method:** Single-loop PID Angle Control.  
* **Balancing Axis:** Roll Axis (Lateral/Side-to-Side Tilt).  
* **Main Code File:** Self\_Balancing\_Bot.ino (Located in the Self\_Balancing\_Bot/ folder).

## **2\. Hardware Requirements**

| Component | Function | Notes |
| :---- | :---- | :---- |
| **Microcontroller** | Arduino Uno / Nano | Used for running the control loop. |
| **IMU Sensor** | MPU6050 (Accel & Gyro) | Provides angle data via the DMP. |
| **Motor Driver** | L298N Dual H-Bridge | Handles 12V power for the motors. |
| **Motors & Power** | 2x 12V DC Gear Motors, 12V Battery | High torque recommended. |

### **2.1. Wiring Connection Summary**

The critical connections are:

* **MPU6050 I²C/Interrupt:** SDA (A4), SCL (A5), INT (D2).  
* **L298N Motor Control (PWM/Direction):** ENA (D5), ENB (D9), IN1-IN4 (D6, D7, D10, D11).

**NOTE:** A circuit schematic/diagram will be added to the docs/ folder soon to visualize these connections.

## **3\. Software Requirements**

The project uses standard Arduino libraries for stability and motion processing.

1. **I2Cdevlib (by Jeff Rowberg):** Required for MPU6050 communication and utilizing the Digital Motion Processor (DMP).  
2. **Arduino PID Library (by Brett Beauregard):** Provides the robust PID control system functions (PID\_v1.h).

## **4\. Key Files and Code Explanation**

| File Path | Explanation |
| :---- | :---- |
| **Self\_Balancing\_Bot/Self\_Balancing\_Bot.ino** | This is the heart of the project. It handles MPU initialization, PID setup, angle reading (Roll axis), PID computation, and translating the resulting signed output into motor PWM signals and direction control via the L298N. |
| **Self\_Balancing\_Bot/PID\_v1.h** | The header for the PID library, defining the PID class and its methods (SetTunings, SetMode, Compute). |
| **MPU6050/ & I2Cdev/ folders** | These folders contain the source code and headers for the MPU6050 and I2C communication libraries. |
| **docs/ (Future)** | This folder will hold the detailed **self\_balancing\_bot\_documentation.md** file, including full tuning guides, code breakdown, and the circuit schematic. |

## **5\. Calibration (Critical Tuning Values)**

The stability requires careful tuning of these constants:

* **Setpoint:** 183.67 (The required Roll angle for perfect vertical balance).  
* **Proportional Gain (**$\\mathbf{K\_p}$**):** 45 (Primary control force).  
* **Derivative Gain (**$\\mathbf{K\_d}$**):** 3.5 (Damping/Braking force to reduce oscillation).  
* **Integral Gain (**$\\mathbf{K\_i}$**):** 100 (Correction for long-term drift or steady-state error).

## **6\. Further Reading**

For a comprehensive technical breakdown of the code logic, control theory, and hardware specifics, refer to:

* [**Detailed Technical Documentation**](https://github.com/tomsononline/self-balancing-robot/blob/main/docs/Two-Wheeled%20Self-Balancing%20Robot%20(Angle%20Control).md))
