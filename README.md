# MSP430 Ultrasonic Capture Firmware

## Motivation

This project demonstrates deterministic pulse-width measurement using hardware timer capture interrupts on the MSP430 platform.

The goal is to model embedded best practices:
- Interrupt-driven architecture
- Non-blocking main loop
- No dynamic allocation
- Clean HAL/driver separation
- Deterministic timing

## Hardware
- MSP430 Launchpad
- HC-SR04 Ultrasonic Sensor
- Resistor divider for 5V -> 3.3V echo line

## Architecture
HAL -> Driver -> Application

## Project Phases
- Timer + interrupt setup
- Echo pulse capture
- Distance calculation
- UART logging
- Threshold handling