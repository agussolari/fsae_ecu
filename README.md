# Motor Control System Using CANopen

## Overview

This project implements a motor control system using the **CANopen** protocol, designed to interface with an **emDrive** motor driver and control a **brushless motor**. The system is built on an **NXP LPC55S06** microcontroller and is capable of controlling the motor's speed and torque.

## Hardware Components

### 1. **Microcontroller: NXP LPC55S06**
   - The core of the system is based on the NXP LPC55S06 microcontroller, a powerful and energy-efficient MCU designed for high-performance applications.
   - The LPC55S06 features an Arm Cortex-M33 core, dual-core processing capabilities, and integrated security features, making it ideal for precise motor control tasks.

### 2. **Motor Driver: emDrive 150**
   - The system uses the **emDrive** motor driver, which supports the CANopen protocol for robust and reliable communication.
   - The emDrive driver is capable of controlling **brushless motors** in various operating modes, including torque and velocity modes.
   - The driver is configured and controlled via CANopen SDO and PDO messages, allowing for flexible and precise motor management.

### 3. **Motor Brushless: EMRAX 188**
   - The project is designed to work with brushless motors, specifically optimized for the eDrive's capabilities.
   - The motor's speed and torque are controlled via CANopen messages, allowing for real-time adjustments and efficient operation.

## Features

- **CANopen Protocol Support**: The project fully implements the CANopen protocol, enabling reliable communication and control of the motor driver.
- **Torque and Velocity Control**: The system supports both torque and velocity control modes, allowing for flexible operation based on the application's needs.
- **State Machine Management**: The firmware includes a state machine that handles the different operational states of the motor driver, including Power-On, Initialization, Pre-operational, and Operational modes.
- **Error Handling**: The system monitors for errors such as DC overcurrent and can handle these conditions gracefully to protect the hardware.
- **CAN Sniffer**: A CAN sniffer tool is included to monitor and debug CANopen messages in real-time.

## Getting Started

### Prerequisites

- **Toolchain**: To compile and upload the firmware, you will need a compatible toolchain for NXP microcontrollers, such as MCUXpresso IDE.
- **CAN Interface**: A CAN interface is required to communicate with the motor driver. This could be integrated on your development board or an external CAN transceiver.
- **Power Supply**: Ensure that you have an appropriate power supply for both the motor and the motor driver.

### Setup

1. **Clone the Repository**:
   ```bash
   https://github.com/agussolari/fsae_ecu
