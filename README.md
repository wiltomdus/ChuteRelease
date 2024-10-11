# Chute Release Project for High-Power Rockets

## Overview
This project aims to develop a parachute release system for high-power rockets, comprising both hardware and firmware components. The system is designed to manage the deployment of parachutes at specific altitudes during the rocket's descent, ensuring a safe recovery. It utilizes a barometer to track altitude and a servo motor to release the parachute at the designated altitude.

### Features
- **Real-Time Altitude Tracking**: Uses a barometer to continuously track altitude.
- **Altitude-Triggered Deployment**: Parachute deployment triggered by a rotary switch-selected altitude.
- **Kalman Filter Integration**: Smoothing and noise reduction for altitude, velocity, and acceleration data.
- **Flight Stage Detection**: Automatic detection of flight stages (Launch, Ascent, Apogee, Descent, and Landing).
- **Dual Operating Modes**: Supports both development mode (mock data) and production mode (real sensor data).

## Project Structure

### Hardware
The hardware for this project is designed using KiCad. It includes schematics and PCB layouts for the parachute release system.

- **`chute_release.kicad_pcb`**: PCB layout for the parachute release system.
- **`chute_release.kicad_sch`**: Top-level schematic for the system.
- **Component Schematics**: Individual schematics for sensors, MCU, power, and interface: `sensors.kicad_sch`, `mcu.kicad_sch`, `power.kicad_sch`, `interface.kicad_sch`.

### Firmware
The firmware is developed using CircuitPython, a fork of MicroPython, to manage the parachute release logic.

#### Main Components
1. **Barometer Class (`barometer.py`)**: Interfaces with the DPS368 pressure sensor to obtain altitude, pressure, and temperature readings. Configures sensor parameters like pressure and temperature oversampling.

2. **Flight Manager (`flightManager.py`)**: Manages the rocket's flight stages, controlling parachute release based on altitude readings. Utilizes a Kalman filter for data smoothing and supports both development and production modes.

3. **Kalman Filter (`kalman_filter.py`)**: Implements a Kalman filter for noise reduction in altitude, velocity, and acceleration data.

4. **Mock Barometer (`mock_barometer.py`)**: Simulates rocket flight for testing purposes, providing mock altitude, pressure, and temperature data.

5. **Boot Configuration (`boot.py`)**: Configures the CircuitPython filesystem, allowing write access during development and securing it during production.

6. **Main Script (`main.py`)**: Coordinates system setup and flight data logging. Handles battery voltage sensing and generates unique filenames for data logs.

## Getting Started

### Installation
1. Clone this repository and upload the firmware files (`boot.py`, `main.py`, `barometer.py`, `flightManager.py`, etc.) to your CircuitPython board.
2. Assemble the PCB based on the provided KiCad files and connect the required components (barometer, servo motor, etc.).
3. Adjust `flightManager.py` as needed to match your rocket's specifications (e.g., parachute deployment altitude, launch detect altitude, etc.).

The pcb can be powered using a USB 5V cable or with a 1S lipo battery(recommanded 400mAh or more).

### Running the System
1. Power on the system and let it detect the launch and track altitude.
2. The parachute will release automatically when the rocket reaches the selected altitude during descent.
3. Flight data is logged to a CSV file for post-flight analysis.

## Ordering the PCB from JLCPCB
To order the PCB from JLCPCB, use the following details to ensure the correct specifications are met:

- **Base Material**: FR-4
- **Layers**: 4
- **Different Design**: 1
- **Delivery Format**: Single PCB
- **PCB Thickness**: 1.6 mm
- **Via Covering**: Plugged
  
Additionnal specifications should be left as default

## License Information

### Hardware License (PCB Designs)
The hardware designs are licensed under the **CERN Open Hardware License - Weakly Reciprocal (CERN OHL-W)**. This license allows usage, copying, modification, and distribution of the hardware designs, provided that any modifications are also shared under the same license.

- **CERN OHL-W Overview**:
  - Allows use and modification of hardware designs.
  - Modified versions must be shared if distributed.
  - Encourages collaboration and freedom to improve designs.

Read the full license terms in the [LICENSE-hardware](./LICENSE-hardware) file.

### Software License (Firmware)
The firmware is licensed under the **GNU Affero General Public License v3 (AGPL v3)**. This license ensures that any software modifications, including those made for network services, must be open source and shared under the same license.

- **AGPL v3 Overview**:
  - Software can be freely used, modified, and distributed.
  - Modified versions must also be distributed under AGPL v3.
  - Source code must be provided if used for network services.

Read the full license terms in the [LICENSE-software](./LICENSE-software) file.
