# 12-DOF Quadruped Robot Dog

A personal DIY robot dog project built for fun during uni! It runs on an **STM32G474** board, using custom embedded C/C++ firmware, inverse kinematics, and IMU balance control.

---

## 🛠️ Hardware Setup

* **MCU:** STM32G474RET6 (Arm Cortex-M4)
* **Servos:** 12x micro servos (3 joints per leg: hip roll, hip pitch, knee pitch)
* **Servo Driver:** PCA9685 16-channel 12-bit PWM board over I2C
* **IMU:** MPU-6050 (6-axis gyro + accelerometer) for pitch/roll balance
* **Power:** 2S LiPo / 18650 setup + buck converters stepping down for the servos (5V–6V) and logic (3.3V)
* **Chassis:** 3D printed in PLA

---

## ✨ Features

* **Custom Inverse Kinematics (IK):** Calculates joint angles on the fly from target foot coordinates.
* **Basic IMU Stabilization:** Uses pitch and roll readings from the MPU-6050 to adjust leg extension and keep the body level.
* **I2C Drivers:** Handled via STM32 HAL for the PCA9685 PWM expander and MPU-6050.
* **CMake & CubeMX:** Configured with STM32CubeMX, built cleanly using CMake.

---

## 💻 How to Build & Flash

I use VS Code with the STM32 / Cortex-Debug extensions to build and flash directly:

1. Clone this repo:
   ```bash
   git clone [https://github.com/148Eric/robotdog.git](https://github.com/148Eric/robotdog.git)

2. Open the folder in VS Code.

3. Make sure you have the CMake Tools and Cortex-Debug / STM32 extensions installed.

4. Plug in your ST-Link to your Mac/PC.

5. Just hit Run and Debug (F5) or click the build/run button at the bottom status bar to compile and flash the firmware straight to the STM32!

---

## Mechanical & Attribution

The 3D printed chassis and leg designs are adapted from this awesome Instructables guide:

* Original 3D Model & Guide: https://www.instructables.com/Arduino-Robot-Dog/

Massive shoutout to the original creator for open-sourcing the STL files and mechanical layout!

---

## License

Distributed under the MIT License. Feel free to use, modify, or fork it for your own robotics experiments!
   
