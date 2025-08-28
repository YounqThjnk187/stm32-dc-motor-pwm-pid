# 🚀 DC Motor Speed Control with PWM + PID using STM32

## 📌 Overview
This project demonstrates **closed-loop control of a DC motor** using:
- PWM (Pulse Width Modulation) for speed control
- PID algorithm for precise adjustment under load changes
- STM32F103C8T6 microcontroller + L298N H-Bridge
- Encoder (FC-03) for feedback
- LCD 1602 I2C for displaying speed

The project was originally developed as a **final course project** for the Microprocessor & Microcontroller course (CE103.P21, UIT - University of Information Technology, VNUHCM).

---

## ⚙️ Hardware Setup
- **MCU**: STM32F103C8T6 (Blue Pill)
- **Motor Driver**: L298N
- **DC Motor**: TT Gear Motor with Encoder FC-03
- **Display**: LCD 1602 I2C
- **Power Supply**: 12V DC, 2A

Block Diagram:

[STM32] --PWM--> [L298N] --> [DC Motor + Encoder]
<--Feedback-- FC-03
--I2C--> [LCD 1602]


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
.
├─ docs/ # Reports and presentation
├─ src/ # Firmware code (STM32CubeIDE / HAL)
└─ README.md # Project documentation

