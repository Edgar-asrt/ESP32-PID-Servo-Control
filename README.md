# SISTEM-OF-CONTROL
PID-based position control system for a modified servo motor using an ESP32. The original servo controller was removed, the feedback potentiometer was connected directly to the microcontroller ADC, and the DC motor is driven through an H-bridge. The system supports real-time PID tuning through potentiometers or SCADA/LabVIEW communication via UART.

# ESP32 PID Servo Position Control

## Overview

This project implements a closed-loop position control system for a modified servo motor using an ESP32.

The original servo electronics were removed, exposing the DC motor and feedback potentiometer. The potentiometer is connected directly to the ESP32 ADC inputs while the motor is driven through an external H-bridge.

A PID controller regulates the shaft position using feedback from the potentiometer. Control parameters can be adjusted locally through potentiometers or remotely through a SCADA/LabVIEW interface over UART.

## Features

- Closed-loop position control
- PID controller (P, I and D gains)
- Real-time tuning through analog potentiometers
- SCADA/LabVIEW communication via UART
- Bidirectional motor control using H-bridge
- OLED SSD1306 user interface
- FreeRTOS multitasking architecture
- Adjustable setpoint control
- Disturbance injection for control system testing

## Hardware

- ESP32
- Modified servo motor
- H-bridge motor driver
- Feedback potentiometer
- SSD1306 OLED display
- Control potentiometers
- Mode selection switch

## Software Architecture

The firmware is built using FreeRTOS and consists of several concurrent tasks:

- PID Task
- ADC Acquisition Task
- Potentiometer Reading Task
- UART Communication Task
- OLED Display Task

## Control Strategy

The controller computes:

PID Output = Kp·e(t) + Ki·∫e(t)dt + Kd·de(t)/dt

where:

- e(t) = Setpoint − Position
- Position is obtained from the servo feedback potentiometer
- Output drives the H-bridge PWM channels

Additional features include:

- Anti-windup protection
- Derivative filtering
- Bidirectional motor con
