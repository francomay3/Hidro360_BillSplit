# ESP32 Learning Sandbox

A personal learning project for exploring ESP32 capabilities, including:

- DS18B20 temperature sensor integration
- GPIO button handling
- LED control and blinking patterns

## Features

- Temperature reading using DS18B20 sensor
- Button input with LED feedback
- Periodic LED blinking demonstration
- FreeRTOS task management

## Setup Requirements

- ESP-IDF framework
- VS Code with ESP-IDF extension
- ESP32 development board
- DS18B20 temperature sensor
- Push button and LEDs

## Hardware Connections

- Temperature Sensor: GPIO 26
- Button: GPIO 32
- Button LED: GPIO 33
- Blink LED: GPIO 25

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

## Project Status

This is a learning sandbox for experimenting with ESP32 features and peripherals.
