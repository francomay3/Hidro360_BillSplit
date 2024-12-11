# Hidro360 - Bill Split

**Hidro360 Bill Split** is a proof-of-concept system designed to connect multiple sensor modules to a central gateway for data collection, local storage, and cloud logging. The repository contains two primary projects:

1. **ESP-IDF Project (Module Side):** Runs on ESP32-based sensor modules.
2. **Python Project (Gateway Side):** Runs on a Raspberry Pi gateway.

Though currently in early stages, the ultimate goal is to have multiple modules connected through LoRa and RS485 links to a gateway that processes and logs the data both locally and in the cloud.

## Overview

### Hardware Setup

- **Modules (ESP32):**
  - Uses an ESP32 microcontroller.
  - Equipped with a DS18B20 temperature sensor.
  - Communication via:
    - LoRa using the RA-01 module.
    - RS485 (not yet implemented).
- **Gateway (Raspberry Pi 5):**
  - Also uses an RA-01 LoRa module to receive data.
  - Will listen on two channels:
    - LoRa
    - RS485 (to be implemented)

### Data Flow

1. Each module reads temperature data from its DS18B20 sensor.
2. Data is sent to the gateway over LoRa (and, in the future, RS485).
3. The gateway receives and interprets the incoming packets.
4. The gateway’s data handler stores the readings in a local database and logs them to a cloud-based database.

### Future Plans

- **Custom Protocol:** Implement a more efficient communication protocol.
- **Master-Slave Relationship:** Establish a structured relationship where the gateway acts as a master and modules as slaves.
- **RS485 Channel:** Integrate and fully support RS485 communication.

## Repository Structure

The repository contains a single code workspace (mono.code-workspace) that organizes the two sub-projects:

- **Module Project (ESP-IDF):** Located in its own directory. This project is responsible for reading sensor data and sending it over LoRa (and eventually RS485).

- Gateway Project (Python):
  Also in its own directory. This project will eventually contain the Python scripts for receiving, interpreting, and logging data. (Not fully implemented yet.)

## Getting Started

### Prerequisites

- **ESP-IDF:** Required for building the module firmware for the ESP32.
- **VSCode or Cursor:** Recommended for a unified development environment.
- **Raspberry Pi 5:** For running the gateway.
- **LoRa Modules (RA-01):** For wireless communication.
- **DS18B20 Sensor:** For temperature measurement on the modules.

### Steps to Start Development

1. Clone the Repository:

```bash
git clone https://github.com/francomay3/Hidro360_BillSplit.git
```

2. Open the Workspace:
   In VSCode or Cursor, go to `File` => `Open Workspace from File...` and select mono.code-workspace.

#### Module Project Setup (ESP-IDF):

1. Install ESP-IDF following the official ESP-IDF documentation (e.g., using the ESP-IDF installer or setting up the environment manually).
2. In the module project directory, run:

   ```bash
   Copy code
   get_idf
   idf.py build
   ```

   This will fetch the necessary dependencies and build the firmware for the ESP32 modules.

#### Gateway Project Setup (Python):

- The Python script and environment setup are not yet ready.
- A custom script and instructions will be provided soon to simplify starting work on the gateway project.

## Contributing and Next Steps

- The project is in an early proof-of-concept stage.
- Contributions, feedback, and suggestions are welcome.
- As the project evolves, more documentation and step-by-step guides will be provided.
- The protocol will be defined in this file for use as documentation.
