# System Architecture Overview
This project is an interrupt-driven ultrasonic distance measurement system built on the MSP430FR5994.

The system:
- Periodically triggers an HC-SR04 ultrasonic sensor
- Measures echo pulse width with microsecond precision
- Converts pulse width to distance (cm)
- Maps distance to a blink period
- Uses a hardware timer interrupt to blink an LED
- Displays detection state and last measured distance on an LCD
- Sleeps between measurements to avoid wasting CPU cycles

The architecture is timer-driven and event-based.

---
## Clock Architecture
Clock | Used For | Why
:--- | :--- | :---
SMCLK (~1 MHz) | Microsecond timing | High resolution for echo pulse
ACLK (~32.768 kHz) | Periodic scheduling | Low-power, stable, long intervals

---
## Timer Config & Responsibilities

### TA1 - Free-Running Timebase
- Source: SMCLK
- Mode: Continuous
- Purpose:
    - Microsecond delays (`delay_us`)
    - Echo pulse width measurement (`TA1R` subtraction)

Runs continuously as a 16-bit timestamp counter, providing a deterministic time measurement independent of main loop execution time.

### TA0 - LED Blink Scheduler
- Source: ACLK
- Mode: Up
- Trigger: CCR0 interrupt
    - ISR toggles LED using XOR
- Purpose:
    - Generate period interrupts
    - Decouple blink timing from measurement logic
    - Allow dynamic period updates without resetting phase

Lightweight and independent blink ISR.

### TA2 - Measurement Scheduler
- Source: ACLK
- Mode: Up
- Trigger: CCR0 interrupt
    - ISR sets `measure_due = true`
    - ISR exits low power mode
- Purpose:
    - Provide stable periodic measurement cadence
    - Wake CPU from LPM0

Avoids polling and enables event driven architecture.

---
## Measurement Flow

### 1. TA2 Interrupt Fires (Periodic Heartbeat)
- Sets `measure_due = true`
- Wakes CPU from LPM0

### 2. Main Loop Wakes
Main does:
```c
while (!measure_due) sleep;
measure_due = false; 
```
Then:
- Calls `measure_echo_pulse()`
- Converts pulse width -> cm
- Updates blink period

Then returns to sleep.

### 3. Ultrasonic Trigger Sequence
Inside `measure_echo_pulse()`:
- Reset state atomically
- Arm Port 3 interrupt for rising edge
- Send 10 µs trigger pulse

Sensor emits ultrasonic burst.

### 4. Port Interrupt (Echo Rising Edge)
- ISR records `echo_start = TA1R`
- Switches interrupt to falling edge
- Sets state to wait for falling edge

Hardware edge detection ensures microsecond accuracy.

### 5. Port Interrupt (Echo Falling Edge)
- ISR reads `end = TA1R`
- Computes `echo_width_us = end - echo_start`
- Sets `echo_got_pulse = true`
- Switches back to rising edge for next cycle

Simple 2-state edge machine.

### 6. Distance Conversion
Main computes:
```c
distance_cm = pulse_us / 58
```
Then:
- Clamps to a valid range
- Maps to blink period
- Calls `blink_start_or_update_ms()`

### 7. LED Blink
TA0 ISR toggles LED:
```c
LED_OUT ^= LED_PIN;
```
Blinking is fully asynchronous from measurement.

### 8. LCD Render
LCD display is updated based on `{ok, cm}` after each measurement cycle.
Driven by a small state machine:
- `LCD_STATE_NO_OBJECT`  
  Display: “No object / detected!”
- `LCD_STATE_TOO_CLOSE`  
  Display: “Object too / close!”
- `LCD_STATE_DETECTED`  
  Display: “Object: <cm> cm / Blinking”

### 9. Repeat
Return to LPM0 until next TA2 tick.

---