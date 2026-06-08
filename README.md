# Edge-AI Flight Controller: ESP32 + MAX78000

This repository contains the firmware and hardware design files (KiCad) for a custom-built, autonomous quadcopter flight controller. 

Initially prototyped as a distributed AVR/ESP32 system, the architecture has been heavily optimized into a pure ESP32 FreeRTOS environment to eliminate serial latency. High-frequency flight dynamics, sensor fusion (SPI IMU), and motor mixing run deterministically on Core 1, while Core 0 handles live UDP telemetry and inter-chip communication.

A major feature of this flight stack is the integration of Edge AI. A **MAX78000** microcontroller with a dedicated hardware Convolutional Neural Network (CNN) accelerator is used as an onboard vision co-processor. Paired with a **Seeed Studio camera** for live FPV and visual data ingestion, the MAX78000 runs ultra-low-latency object tracking inference directly at the edge, feeding real-time coordinate data back to the ESP32 for autonomous flight corrections—all without relying on an external PC for video processing.

⚠️ CAUTION AND LEGAL DISCLAIMER ⚠️

Potential for Misuse: Due to the advanced tracking and autonomous capabilities of this platform, the underlying technology could theoretically be weaponized.
Strict Prohibition: Do NOT attempt to weaponize or modify this project for offensive purposes without explicit legal authority, proper licensing, and strict regulatory compliance.
Intended Use: This project is strictly intended for educational research and professional portfolio-building purposes only.

Whole Documentation will be uploaded soon after everything is finished, including the bare-metal code (programmed in C language), circuit PCB Schematics, some necessary footage, and the Project Paper.
