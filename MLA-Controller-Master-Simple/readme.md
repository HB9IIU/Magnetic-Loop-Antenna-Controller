
# MLA-Controller-Master-Simple

This repository contains the code for the **simplified version** of the MLA-Controller-Master. It is designed for users who want an easy, plug-and-play setup without the need for soldering or complex hardware connections.

## Key Features

- **Simplified Design**: Optimized for use with the **ESP32 Cheap Yellow Board** and its accompanying **2.2-inch 240x320 display**.
- **Core Functionality**: Provides the same essential functions as the original MASTER:
  - Retrieves frequency data from the transceiver.
  - Communicates wirelessly with the SLAVE device.
  - Controls the tuning capacitor of the Magnetic Loop Antenna.
- **No Touch Buttons**: Due to the smaller screen size, touch buttons for manual frequency selection are not included.

## Why This Version?

The **MLA-Controller-Master-Simple** was created to make the MLA system more accessible to beginners or those unfamiliar with soldering. By leveraging the **ESP32 Cheap Yellow Board**, this version eliminates the need for custom hardware connections. Users can simply flash the code and start using it.

## Hardware Requirements

- **ESP32 Cheap Yellow Board** with a **2.2-inch 240x320 display**:
  - Refer to the [ESP32-Cheap-Yellow-Display GitHub repository](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display) for more information.
- Power supply for the ESP32 and the display.
- Wireless communication setup with the SLAVE device (same as the original MASTER).

## Installation

1. **Flash the Code**:
   - Download the code from this repository.
   - Flash it onto the ESP32 Cheap Yellow Board using [PlatformIO](https://platformio.org/) or the Arduino IDE.

2. **Setup**:
   - Ensure the ESP32 Cheap Yellow Board is powered and connected to your system.
   - The display will automatically show the tuning status and frequency information.

3. **Operation**:
   - The MASTER-Simple retrieves the transceiver frequency via Bluetooth and communicates with the SLAVE to adjust the tuning capacitor.

## Differences from the Original MASTER

- **Display Size**: The original MASTER uses a **4-inch 480x320 TFT display**, whereas this version uses a smaller **2.2-inch 240x320 display**.
- **No Touch Buttons**: Frequency selection buttons are not included in this version due to the smaller screen.

## Documentation

For detailed system information and additional features, refer to the [main repository README](https://github.com/HB9IIU/Magnetic-Loop-Antenna-Controller/tree/main).

## Acknowledgments

This version leverages the excellent work from the [ESP32-Cheap-Yellow-Display repository](https://github.com/witnessmenow/ESP32-Cheap-Yellow-Display). Special thanks to the contributors for making this hardware integration seamless.



