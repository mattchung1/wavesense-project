# WaveSense: Haptic Navigation for the Visually Impaired

![Zephyr RTOS](https://img.shields.io/badge/OS-Zephyr%20RTOS-blue)
![Hardware](https://img.shields.io/badge/Hardware-nRF5340-green)
![Status](https://img.shields.io/badge/Status-Prototype-orange)

WaveSense is an assistive technology wearable designed to enhance spatial awareness for visually impaired individuals. By converting environmental data—distance and temperature—into intuitive haptic feedback, the device provides a "sixth sense" for navigation and safety.

## Project Demo

Check out the device in action:

[![Watch the Demo](https://img.youtube.com/vi/ASVKNsmMCBw/0.jpg)](https://youtube.com/shorts/ASVKNsmMCBw)

> *Click the image above to watch the demonstration video.*

## Project Files
PCB view for the Wavesense

![PCB Layout View](wavesense-project/altium/samples/Wavesense.png)

SCH View for the Wavesense



## Repository Structure

This repository is organized into two primary engineering domains:

```text
WaveSense/
├── firmware/          # Embedded C code built on the Zephyr RTOS (nRF Connect SDK)
│   ├── src/           # Application source code (main.c, sensor logic)
│   ├── boards/        # Custom board overlays and configuration
│   └── prj.conf       # Kernel configuration (sensor drivers, logging, RTT)
│
└── altium/            # Hardware Design Files
    ├── examples/    # Circuit design and connectivity    
    └── .Sch / .Pcb    # Component placement and routing
```

## Key Features

The system utilizes a dual-haptic feedback loop to communicate two distinct environmental factors:

1.  **Proximity Navigation (ToF Sensor):**
    * Utilizes a **VL53L0X Time-of-Flight sensor** to measure distance to obstacles.
    * **Inverse Haptic Mapping:** As the user's hand gets closer to an object, the vibration intensity of the primary motor increases linearly, mimicking the sensation of "touching" the object remotely.

2.  **Thermal Safety Alert (IR Array):**
    * Utilizes a **Panasonic AMG8833 IR Array** (Grid-EYE) to detect heat sources.
    * **Threshold Trigger:** If an object in the field of view exceeds a safety threshold (e.g., a hot stove or machinery), a secondary vibration motor triggers a distinct alert pattern.

## Hardware Specifications

* **Microcontroller:** Nordic Semiconductor nRF5340 (Dual-core ARM Cortex-M33)
* **Operating System:** Zephyr RTOS / nRF Connect SDK (v2.9.0)
* **Sensors:**
    * STMicroelectronics VL53L0X (ToF Distance)
    * Panasonic AMG8833 (8x8 IR Temperature Array)
* **Actuators:** 2x ERM/LRA Vibration Motors (PWM controlled via MOSFET drivers)

## Pinout Configuration

| Component | nRF5340 Pin | Bus/Protocol |
| :--- | :--- | :--- |
| **I2C SDA** | P0.26 | I2C1 |
| **I2C SCL** | P0.27 | I2C1 |
| **Haptic Motor 1 (Dist)** | P0.05 | PWM0 |
| **Haptic Motor 2 (Temp)** | P0.06 | PWM2 |

## Getting Started

### Prerequisites
* [nRF Connect SDK](https://www.nordicsemi.com/Software-and-tools/Software/nRF-Connect-SDK)
* Visual Studio Code with nRF Connect Extension

### Build & Flash
To build the firmware for the nRF5340 DK:

1.  Open the `firmware` folder in VS Code.
2.  Add a build configuration for `nrf5340dk_nrf5340_cpuapp`.
3.  Run a pristine build.
4.  Flash to the device using `west flash` or the nRF Connect sidebar.

```bash
cd firmware
west build -b nrf5340dk_nrf5340_cpuapp
west flash
