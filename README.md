# Chute Release Project for High-Power Rockets

## Overview
This project involves the development of a parachute release system for high-power rockets. The system includes both hardware and firmware components, designed to manage the deployment of parachutes at specific altitudes during the rocket's descent phase. The system leverages a barometer to track altitude, and a servo motor to release the parachute when the correct altitude is reached.

### Features:
- Real-time altitude tracking using a DPS310 barometer.
- Parachute deployment triggered by a rotary switch-selected altitude.
- Kalman filter for altitude smoothing and noise reduction.
- Flight stages detection (Launch, Ascent, Apogee, Descent, and Landed).
- Both development (mock data) and production (real sensor data) modes.

## Project Structure

### Hardware
The hardware design files for this project were created using KiCad. These include the schematic and PCB layouts for the parachute release system.

- `chute_release.kicad_pcb`: KiCad PCB layout for the parachute release system.
- `chute_release.kicad_sch`: Schematic design for the parachute release system.
- Other related schematics: `sensors.kicad_sch`, `mcu.kicad_sch`, `power.kicad_sch`, and `interface.kicad_sch`.

### Firmware
The firmware is written in CircuitPython and manages the barometer readings, parachute deployment, and flight stage tracking.

#### Main Components:
1. **Barometer Class (`barometer.py`)**
   - Interacts with the DPS310 sensor to obtain altitude, pressure, and temperature data.
   - Initializes the barometer and configures sensor parameters like pressure and temperature oversampling.
   
2. **Flight Manager (`flightManager.py`)**
   - Manages flight stages from launch to landing.
   - Controls the parachute release based on altitude readings.
   - Utilizes Kalman filters to smooth altitude, velocity, and acceleration data.
   - Can operate in development mode with simulated data or in production mode with real sensor data.

3. **Kalman Filter (`kalman_filter.py`)**
   - Implements a simple Kalman filter to reduce noise in altitude, velocity, and acceleration readings.

4. **Mock Barometer (`mock_barometer.py`)**
   - Simulates a model rocket flight for testing purposes.
   - Provides simulated altitude, pressure, and temperature data for development mode.

5. **Boot Configuration (`boot.py`)**
   - Configures the CircuitPython filesystem based on USB connection status.
   - Remounts the filesystem to allow write access during development and locks it during production.

6. **Main Script (`main.py`)**
   - Coordinates the overall system setup and flight data logging.
   - Handles battery voltage sensing and generates unique filenames for flight data.

## Getting Started

### Prerequisites
- A CircuitPython-compatible microcontroller (e.g., RP2040-based board).
- KiCad for viewing and editing hardware design files.
- A DPS310 barometer for altitude measurements.
- A servo motor for controlling the parachute release mechanism.

### Installation
1. Clone this repository and upload the firmware files (`boot.py`, `main.py`, `barometer.py`, `flightManager.py`, etc.) to your CircuitPython board.
2. Assemble the PCB based on the provided KiCad files and connect the required components (barometer, servo motor, etc.).
3. Adjust the `flightManager.py` file if needed to match your rocket's specifications (e.g., altitude for parachute release).

### Running the System
1. Power on the system and allow it to detect the launch and track altitude.
2. The parachute will be released automatically when the rocket reaches the selected altitude during descent.
3. Flight data will be logged to a CSV file for further analysis.

## License Information

### Hardware License (PCB Designs)
The hardware designs in this repository are licensed under the **CERN Open Hardware License - Weakly Reciprocal (CERN OHL-W)**. This license allows anyone to use, copy, modify, and distribute the hardware design files, provided that any modified versions are also made available under the same license.

- **CERN OHL-W Overview:**
  - Allows you to use and modify the hardware.
  - Modifications must be shared if distributed.
  - This license is designed for hardware projects and ensures freedom to use and improve designs while encouraging collaboration.

You can read the full license terms in the [LICENSE-hardware](./LICENSE-hardware) file.

### Software License (Software)
The software (firmware) in this repository is licensed under the **GNU Affero General Public License v3 (AGPL v3)**. This license ensures that any modifications to the software, including modifications made for network services, must also be open source and licensed under the AGPL v3.

- **AGPL v3 Overview:**
  - The software can be freely used, modified, and distributed.
  - Any modified versions must also be distributed under AGPL v3.
  - If the software is used to provide a network service, the source code of the modified version must be made available to users of the service.

You can read the full license terms in the [LICENSE-software](./LICENSE-software) file.
