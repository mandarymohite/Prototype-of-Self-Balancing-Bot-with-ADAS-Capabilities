# Self Balancing Robot with ADAS Prototype

This project is a real-time embedded self-balancing robot built on Raspberry Pi.  
It uses MPU6050 IMU data and a PID controller to keep the robot upright, while also adding a simple ADAS layer for obstacle detection and safety.

## Features
- Self-balancing using accelerometer and gyroscope data
- Complementary filter for tilt estimation
- PID-based motor control
- Autonomous emergency stop
- Forward collision warning
- Adaptive speed limiting
- Basic lane/corridor keeping using ultrasonic sensors

## Hardware
- Raspberry Pi
- MPU6050 IMU
- TB6612FNG motor driver
- 2 DC motors
- HC-SR04 ultrasonic sensors

## Files
- `mySegway.c` – main control loop and real-time threads
- `motors.c` – motor control functions
- `adas.c` / `adas.h` – ADAS logic and state handling
- `Makefile` – build setup
