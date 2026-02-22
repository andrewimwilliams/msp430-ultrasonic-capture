# MSP430 Ultrasonic Distance Measurement Firmware

## Motivation

This project demonstrates deterministic pulse-width measurement using hardware timer capture interrupts on the MSP430 platform, with real-time distance visualization on an I2C LCD display.

The goal is to model embedded systems best practices:
- Interrupt-driven architecture
- Non-blocking main loop
- Deterministic timing using hardware peripherals
- No dynamic memory allocation
- Clean HAL -> Driver -> Application separation
- Clear separation between timing-critical logic and presentation layer

---
## Hardware
![Hardware Setup](images/ultrasensor_setup.jpg)
- MSP-EXP430FR5994 Launchpad
- HC-SR04 Ultrasonic Sensor
- 16x2 HD44780 LCD (I2C backpack)
- Resistor divider (5V -> 3.3V) on echo line

---
## Core Design Principles
### 1. Deterministic Measurement
Echo pulse width is measured using hardware timer capture interrupts, no polling loops or software timing.

### 2. Interrupt-Driven Timing
- Capture ISR records rising and falling edges of the echo pulse.
- Blink timing is driven by timer compare interrupts.
- ISRs remain short and bounded.

### 3. Non-Blocking Main Loop
The main loop:
- Checks for completed measurements
- Updates system state
- Updates LCD only when necessary
No busy-wait delays are used in the application layer.

### 4. Presentation Layer Separation
LCD updates occur strictly in the main loop and never inside interrupts, ensuring:
- Measurement timing is never disturbed
- I2C transactions do not impact real-time capture performance
- The system remains scalable and maintainable

---
## Features
- Accurate distance measurement in centimeters
- Threshold-based LED blinking behavior
- Real-time distance display on 16x2 LCD
- Cleanly separated hardware abstraction
- Fully interrupt-driven timing model

---
## Build & Flash
```
make
make flash
```

---
## Documentation
- `docs/architecture.md` - detailed system architecture and process flow
- `docs/wiring.md` - high-level wiring diagram

---