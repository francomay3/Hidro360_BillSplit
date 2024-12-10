# ESP32 Learning Sandbox

A personal learning project for exploring ESP32 capabilities, including:

- DS18B20 temperature sensor integration

## Features

- Temperature reading using DS18B20 sensor

## Setup Requirements

- ESP-IDF framework
- VS Code with ESP-IDF extension
- ESP32 development board
- DS18B20 temperature sensor

## Hardware Connections

- Temperature Sensor: GPIO 26

## Building and Running

1. Find your ESP32's port:

   - On MacOS/Linux:
     ```bash
     ls /dev/cu.usbserial*
     ```
   - On Windows:
     ```bash
     # Check Device Manager under "Ports (COM & LPT)"
     # It will appear as "Silicon Labs CP210x" or similar
     # Example: COM3
     ```
   - On Linux it might also appear as:
     ```bash
     ls /dev/ttyUSB*
     ```

2. Source ESP-IDF environment:

   ```bash
   . $HOME/esp/esp-idf/export.sh
   ```

3. With the esp32 connected to the computer, run:

   ```bash
   idf.py -p /dev/cu.usbserial-0001 flash monitor
   ```

   Replace `/dev/cu.usbserial-0001` with your port from step 1

4. Exit monitor with `Ctrl+T` followed by `Ctrl+X`

## Adding New Dependencies

To add new source files to the project:

1. Create header file in main folder:
   ```bash
   touch main/newFile.h
   ```

2. Create source file in main folder:
   ```bash
   touch main/newFile.cpp
   ```

3. Add the source file to `main/CMakeLists.txt`:
   ```cmake
   idf_component_register(
       SRCS "main.cpp" "newFile.cpp"  # Add your new source file here
       INCLUDE_DIRS "."
   )
   ```

4. Include the header in your main entry file:
   ```cpp
   #include "newFile.h"
   ```

## Project Status

This is a learning sandbox for experimenting with ESP32 features and peripherals.

