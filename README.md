# WaveSense: Haptic Navigation for the Visually Impaired

![Zephyr RTOS](https://img.shields.io/badge/OS-Zephyr%20RTOS-blue)
![Hardware](https://img.shields.io/badge/Hardware-nRF5340-green)
![Status](https://img.shields.io/badge/Status-Prototype-orange)

WaveSense is an assistive technology wearable designed to enhance spatial awareness for visually impaired individuals. By converting environmental data—distance and temperature—into intuitive haptic feedback, the device provides a "sixth sense" for navigation and safety.

## Project Demo

Check out the device in action:

[![Watch the Demo](https://img.youtube.com/vi/ASVKNsmMCBw/0.jpg)](https://youtube.com/shorts/ASVKNsmMCBw)

> *Click the image above to watch the demonstration video.*

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
