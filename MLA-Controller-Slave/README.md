
# MLA-Controller-Slave

This repository contains the code for the **ESP32 microcontroller**, functioning as the **SLAVE** device in the Magnetic Loop Antenna (MLA) Controller system. The SLAVE is responsible for precise control of the MLA tuning capacitor using a stepper motor and gear reduction system.

<div style="text-align: center;">
    <a href="https://www.youtube.com/watch?v=OqZV3zAZ8kQ" target="_blank">
        <img src="https://img.youtube.com/vi/OqZV3zAZ8kQ/maxresdefault.jpg" alt="Short Demo" />
    </a>
</div>

[Right-click here and open in a new tab to watch the video](https://www.youtube.com/watch?v=OqZV3zAZ8kQ)

## Overview

The SLAVE device communicates wirelessly with the MASTER ESP32 over an ad hoc 2.4 GHz network. It receives tuning instructions from the MASTER, calculates the required stepper motor adjustments, and ensures the tuning capacitor is positioned accurately to achieve optimal resonance.

## Key Features

- **Stepper Motor Control**: Operates a stepper motor with gear reduction for fine control of the tuning capacitor.
- **Wireless Communication**: Communicates with the MASTER ESP32 via a 2.4 GHz network for seamless coordination.
- **Persistent Position Tracking**: Maintains the capacitor's position in flash memory to ensure accuracy even after power loss.
- **Precise Resonance Tuning**: Works in tandem with the MASTER to achieve and maintain precise resonance for high-Q antennas.

## Stepper Motor Configuration and Resolution

The **DM542 stepper motor driver** is configured with a **microstep resolution of 64 microsteps per step**, achieved by setting **SW6 = OFF** and **SW7 = OFF**. With a standard 1.8° stepper motor, this results in:

```
Steps per Revolution = Microsteps per Step × Steps per Revolution (1.8° motor)
                     = 64 × 200
                     = 12,800 steps/revolution
```

The stepper motor is paired with a **1:10 gear reduction**, which multiplies the precision of the motor. The combination of microstepping and gear reduction results in a **final resolution** of:

```
Final Resolution = Steps per Revolution × Gear Ratio
                 = 12,800 × 10
                 = 128,000 steps/revolution
```

### Example: Moving from 7 MHz to 14 MHz

In practical terms, moving the tuning capacitor from **7 MHz to 14 MHz** requires approximately **1,000,000 steps**, which translates to:

```
Number of Turns = Total Steps ÷ Final Resolution
                = 1,000,000 ÷ 128,000
                ≈ 7.81 turns
```

---

## Stepper Motor Driver Connections

The SLAVE uses the DM542 stepper motor driver (or compatible) to control the stepper motor. Below are the pin connections:

- **Pulse Pin (PUL)**:
  - ESP32 Pin: `23`
  - DM542 Terminal: `PUL+` (Pulse signal input)
  - Function: Sends step signals to the driver, causing the motor to move one step per pulse.

- **Direction Pin (DIR)**:
  - ESP32 Pin: `22`
  - DM542 Terminal: `DIR+` (Direction signal input)
  - Function: Sets the motor's rotation direction (clockwise or counterclockwise).

- **Enable Pin (EN)**:
  - ESP32 Pin: `21`
  - DM542 Terminal: `EN+` (Enable signal input)
  - Function: Enables or disables the motor driver. A `LOW` signal enables the driver.

### Wiring Notes

- **Ground Connections**:
  - Connect the ESP32's `GND` to the `GND` of the DM542 (`PUL-`, `DIR-`, and `EN-` are typically connected to the driver's ground).

- **Power Supply**:
  - Ensure the DM542 is powered with an appropriate voltage for the stepper motor (typically 20-50VDC). The motor's voltage does not come from the ESP32.

### Alternative Driver

The **DM542** was chosen because it was available in my collection of spare parts, but a simpler and more compact driver, such as the **DRV8825**, could also perform this task adequately. The DRV8825 offers lower cost and size advantages, but the DM542 provides superior handling of noise, smoother operation, and higher torque at higher speeds.

---

## Important Notes

- **Included Libraries**: All required libraries are already included in the `lib` folder. There is no need to install additional libraries.
- **Recommended IDE**: This code is designed to be compiled using [PlatformIO](https://platformio.org/) in Visual Studio Code. PlatformIO offers an efficient and streamlined environment for ESP32 development.

## Documentation

For more information about the system's functionality, initial setup, and additional features, please refer to the [main repository README](https://github.com/HB9IIU/Magnetic-Loop-Antenna-Controller/tree/main).
