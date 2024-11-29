
# MLA-Controller-Slave

This repository contains the code for the **ESP32 microcontroller**, functioning as the **SLAVE** device in the Magnetic Loop Antenna (MLA) Controller system. The SLAVE is responsible for precise control of the MLA tuning capacitor using a stepper motor and gear reduction system.

## Overview

The SLAVE device communicates wirelessly with the MASTER ESP32 over an ad hoc 2.4 GHz network. It receives tuning instructions from the MASTER, calculates the required stepper motor adjustments, and ensures the tuning capacitor is positioned accurately to achieve optimal resonance.

## Key Features

- **Stepper Motor Control**: Operates a stepper motor with gear reduction for fine control of the tuning capacitor.
- **Wireless Communication**: Communicates with the MASTER ESP32 via a 2.4 GHz network for seamless coordination.
- **Persistent Position Tracking**: Maintains the capacitor's position in flash memory to ensure accuracy even after power loss.
- **Precise Resonance Tuning**: Works in tandem with the MASTER to achieve and maintain precise resonance for high-Q antennas.

## Stepper Motor Driver Connections

The SLAVE uses the DRV8825 stepper motor driver (or compatible) to control the stepper motor. Below are the pin connections:

- **Pulse Pin (PUL)**:
  - ESP32 Pin: `23`
  - DRV8825 Terminal: `PUL+` (Pulse signal input)
  - Function: Sends step signals to the driver, causing the motor to move one step per pulse.

- **Direction Pin (DIR)**:
  - ESP32 Pin: `22`
  - DRV8825 Terminal: `DIR+` (Direction signal input)
  - Function: Sets the motor's rotation direction (clockwise or counterclockwise).

- **Enable Pin (EN)**:
  - ESP32 Pin: `21`
  - DRV8825 Terminal: `EN+` (Enable signal input)
  - Function: Enables or disables the motor driver. A `LOW` signal enables the driver.

### Wiring Notes

- **Ground Connections**:
  - Connect the ESP32's `GND` to the `GND` of the DRV8825 (`PUL-`, `DIR-`, and `EN-` are typically connected to the driver's ground).

- **Power Supply**:
  - Ensure the DRV8825 is powered with an appropriate voltage for the stepper motor (typically 8V-45V for DRV8825). The motor's voltage does not come from the ESP32.

### Example Wiring Diagram
A detailed wiring diagram will be added later. Meanwhile, experienced developers can refer to the above pin mapping and the DRV8825 documentation to set up their connections.

## Important Notes

- **Included Libraries**: All required libraries are already included in the `lib` folder. There is no need to install additional libraries.
- **Recommended IDE**: This code is designed to be compiled using [PlatformIO](https://platformio.org/) in Visual Studio Code. PlatformIO offers an efficient and streamlined environment for ESP32 development.

## Documentation

For more information about the system's functionality, initial setup, and additional features, please refer to the [main repository README](https://github.com/HB9IIU/Magnetic-Loop-Antenna-Controller/tree/main).
