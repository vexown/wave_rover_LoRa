# wave_rover_LoRa

This project facilitates communication between a base station and a mobile robot (Wave Rover) using [Heltec WiFi LoRa 32 (V3)](https://heltec.org/project/wifi-lora-32-v3/) modules. This setup is ideal for applications requiring long-range, low-power communication, such as remote control, telemetry data transmission, and status monitoring of the mobile robot.

## Table of Contents

- [Hardware Used](#hardware-used)
- [Features](#features)
- [Project Structure](#project-structure)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Hardware Used

The primary hardware component for this project is the **Heltec WiFi LoRa 32 (V3)**. This is a development board based on the ESP32-S3FN8 microcontroller, which includes Wi-Fi, Bluetooth LE, and LoRa capabilities.

### Key Features of Heltec WiFi LoRa 32 (V3):

- **Microprocessor**: ESP32-S3FN8 (Xtensa® 32-bit LX7 dual-core)
- **LoRa Chip**: SX1262
- **Connectivity**: Wi-Fi (802.11 b/g/n), Bluetooth 5 (LE), LoRa
- **Display**: Onboard 0.96-inch 128x64 OLED display
- **Interface**: Type-C USB
- **Power**: Supports 3.7V lithium battery with integrated management system
- **Antenna**: Onboard 2.4GHz metal spring antenna for Wi-Fi/Bluetooth and an IPEX (U.FL) interface for LoRa.

This board is well-suited for IoT applications, providing a robust set of features for communication and control.

## Features

- **Long-Range Communication**: Utilizes LoRa technology for communication over several kilometers (depending on the environment).
- **Dual-Role Capability**: The same code can be configured to run on either the base station or the mobile robot.
- **Telemetry**: The mobile robot can send telemetry data (e.g., battery level, sensor readings, GPS coordinates) back to the base station.
- **Remote Control**: The base station can send commands to the mobile robot to control its movement and actions.
- **Status Display**: The onboard OLED display can be used to show vital information such as connection status, battery level, and incoming/outgoing messages.
