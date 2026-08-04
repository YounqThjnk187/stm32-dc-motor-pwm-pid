# 🚀 DC Motor Speed Control with PWM + PID using STM32

## 📌 Overview
This project demonstrates **closed-loop control of a DC motor** using:
- PWM (Pulse Width Modulation) for speed control
- PID algorithm for precise adjustment under load changes
- STM32F103C8T6 microcontroller + L298N H-Bridge
- Encoder (FC-03) for feedback
- LCD 1602 I2C for displaying speed

---

## ⚙️ Hardware Setup
- **MCU**: STM32F103C8T6 (Blue Pill)
- **Motor Driver**: L298N
- **DC Motor**: TT Gear Motor with Encoder FC-03
- **Display**: LCD 1602 I2C
- **Power Supply**: 12V DC, 2A

---

## 🧠 Control Algorithm
- **PWM** controls motor speed by varying duty cycle.
- **Encoder** provides real-time RPM feedback.
- **PID Controller** adjusts PWM signal to minimize error between setpoint and actual speed.
- Formula:
  
  `u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt`

Where:
- `e(t)` = Speed error (Setpoint - Actual)
- `Kp, Ki, Kd` = PID gains

---

## 📂 Repository Structure
```text
docs/ # Reports and presentation
src/ # Firmware code (STM32CubeIDE / HAL)
doan2/ # File project
README.md # Project documentation
```
---

## 📊 Results
- Smooth DC motor speed control.
- PID maintains stable RPM under varying loads.
- LCD displays real-time setpoint and actual RPM.
