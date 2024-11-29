
# MLA-Controller-Master

This repository contains the code for the **ESP32 microcontroller** functioning as the **MASTER** device in the Magnetic Loop Antenna (MLA) Controller system. The MASTER is equipped with a TFT touchscreen interface, providing users with a graphical interface to control and monitor the system's operations.

## Overview

The MASTER device retrieves the VFO (Variable Frequency Oscillator) frequency from a compatible transceiver (e.g., IC-705 via Bluetooth) and communicates wirelessly with the SLAVE ESP32 device to adjust the tuning capacitor. The graphical interface simplifies interaction, allowing precise control and real-time feedback during operation.

## Key Features

- **TFT Touchscreen**: Provides an intuitive user interface for controlling and monitoring the system.
- **Wireless Communication**: Communicates with the SLAVE ESP32 over an ad hoc 2.4 GHz network for field portability.
- **Seamless Tuning**: Automatically adjusts the MLA tuning capacitor based on the VFO frequency retrieved from the transceiver.
- **Real-Time Feedback**: Displays system status, tuning progress, and SWR readings on the screen.

## Documentation

For more information about the system's functionality, initial setup, and additional features, please refer to the [main repository README](https://github.com/HB9IIU/Magnetic-Loop-Antenna-Controller/tree/main).
