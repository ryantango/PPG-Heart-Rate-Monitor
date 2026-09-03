# Infrared PPG Heart Rate Monitor

## Overview

A custom photoplethysmography (PPG) system designed to
measure heart rate using infrared optical sensing.

## System Architecture

IR LED
-->
Finger
-->
BPW34 Photodiode
-->
MCP6002 Transimpedance Amplifier
-->
Analog RC Filter
-->
Arduino ADC
-->
Digital Signal Processing
-->
Peak Detection
-->
Heart Rate (BPM)
-->
OLED Display

## Hardware

- BPW34 Photodiode
- TSAL6400 IR LED
- MCP6002 Op-Amp
- Arduino Uno
- SSD1306 OLED
- 604 kΩ TIA feedback resistance
- 47 pF compensation capacitor
- 10 kΩ / 1 µF RC low-pass filter

## Software & Tools

- C++
- Arduino IDE
- LTspice
- KiCad
- Python / MATLAB (planned for signal analysis)

## Signal Processing

1. ADC sampling
2. DC baseline removal
3. Digital low-pass filtering
4. Adaptive thresholding
5. Peak detection
6. Beat-to-beat interval calculation
7. BPM averaging

## Current Status

Working Prototype created; integrating potential improvements.

## Future Improvements

- Finger-presence detection
- BPM validation against a commercial pulse oximeter
- Python/MATLAB analysis
- Custom PCB
- Enclosure
