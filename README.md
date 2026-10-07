# RFID-Guided Robot

An STM32-based mobile robot that uses RFID tags for navigation commands and an MPU6050 IMU for heading control and 90°/180° turning.

## Overview

This project is an RFID-guided mobile robot developed using an STM32F103C8T6 microcontroller.

The robot uses RFID tags placed along a path to determine its next movement command. When a recognized tag is detected, the robot stops, identifies the command associated with the tag, performs the required maneuver, and then continues moving.

An MPU6050 IMU is used to measure the robot's heading and control its orientation during straight-line movement and turning.

The main goal of the project was to develop a simple navigation system in which the robot can follow a predefined path using RFID-based commands rather than relying on a conventional line-following system.

## Main Features

- RFID-based navigation
- Command-based movement using RFID tags
- Straight-line heading correction using PI control
- 90° and 180° turns using MPU6050 heading feedback
- Independent control of the left and right motors
- Stop and repeat-command functionality

## Hardware

| Component | Description |
|---|---|
| STM32F103C8T6 | Main microcontroller |
| RC522 | RFID reader |
| MPU6050 | IMU for heading measurement |
| L298 | Dual H-bridge motor driver |
| DC Motors | Left and right drive motors |
| Li-ion Batteries | Power source |
| Boost Converter | Motor driver voltage supply |
| Battery Charger | Battery charging module |

## Navigation Concept

The robot uses RFID tags as navigation commands.

Each recognized RFID tag is associated with a specific action:

| RFID Tag | Command |
|---|---|
| `Turn_Right` | Turn right by 90° |
| `Turn_Left` | Turn left by 90° |
| `Turn_Around` | Turn by 180° |
| `Repeat_Command_Tag` | Repeat the previous movement command |
| `Stop_Command_Tag` | Stop the robot |

During normal movement, the robot continuously monitors its heading using the MPU6050. If the robot deviates from its previous heading, the motor speeds are adjusted to correct the deviation and maintain a straight path.

When an RFID tag is detected, the robot stops and processes the corresponding command before continuing.

## Control System

### Straight-Line Control

A PI-based feedback controller is used to maintain the robot's heading while moving forward.

The measured yaw angle from the MPU6050 is compared with the desired heading. The resulting error is used to adjust the PWM duty cycle of the two motors.

This allows the robot to compensate for small heading deviations caused by differences between the two motors or external disturbances.

### Rotation Control

The MPU6050 is also used during rotation.

For 90° and 180° turns, the robot rotates while monitoring the change in heading. The rotation process stops when the target angle is reached.

The system supports:

- 90° right turns
- 90° left turns
- 180° turns

## RFID Command System

The RC522 module is connected to the STM32 through SPI.

Each RFID tag has a predefined UID stored in the firmware. When a tag is detected, its UID is compared with the known command tags.

The corresponding movement command is then selected based on the detected UID.

The current implementation uses UID-based command recognition.

## Firmware

The firmware is developed using:

- C
- STM32 HAL
- STM32CubeMX
- STM32F1 microcontroller platform

The main application integrates RFID detection, motor control, heading measurement, and movement control.

### Main Software Modules

```text
main.c
│
├── Motor Control
│   ├── Motor_Forward()
│   ├── Motor_Backward()
│   ├── Motor_Stop()
│   ├── Motor_TurnRight()
│   └── Motor_TurnLeft()
│
├── MPU6050 / Yaw
│   └── Update_Yaw()
│
├── Straight-Line Control
│   └── Maintain_Straight_Line_PI()
│
├── Rotation Control
│   └── PerformRotation()
│
└── RFID Navigation
    └── RC522 UID Detection
```text

## Project Structure

RFID_Robot/
├── Core/
│   ├── Inc/
│   │   ├── mpu6050.h
│   │   └── rc522.h
│   └── Src/
│       ├── mpu6050.c
│       ├── rc522.c
│       └── main.c
└── RFID_Robot.ioc

## Development

The project was developed incrementally through individual hardware and control experiments.

The development process included:

1. STM32 peripheral configuration
2. RC522 RFID reader integration
3. RFID UID detection
4. DC motor control using PWM
5. MPU6050 integration
6. Yaw angle calculation
7. Straight-line heading correction
8. 90° and 180° rotation control
9. Integration of RFID commands with robot movement

## Future Improvements

Possible future extensions include:

- Increasing the number of supported RFID commands
- Writing navigation data directly to RFID tags
- Developing a mobile application for creating and configuring robot paths
- Building a map-based navigation interface
- Improving the turning and heading-control algorithms
- Adding more advanced path-planning capabilities
- Developing an improved hardware version of the robot

## Author

**Mahdi Rahpeyma**

Electrical & Electronics Engineer  
Embedded Systems | STM32 | Firmware | PCB Design