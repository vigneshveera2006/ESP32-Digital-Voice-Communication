# ESP32 Digital Voice Communication System

## Project Overview
A real-time wireless digital voice communication system using two ESP32 development boards. The system captures live voice, encodes it using different digital encoding techniques, transmits it wirelessly and reconstructs the audio at the receiver.

## System Architecture

**Transmitter:**
INMP441 Microphone → ESP32 → Digital Encoding → ESP-NOW

**Receiver:**
ESP-NOW → ESP32 → Digital Decoding → MAX98357A Amplifier → Speaker

## Encoding Techniques

| S1 | S2 | Technique |
|---|---|---|
| 0 | 0 | PCM |
| 0 | 1 | DPCM |
| 1 | 0 | DM |
| 1 | 1 | ADM |

A third physical switch controls transmission ON/OFF.

## Hardware Components
- 2 × ESP32 development boards
- INMP441 I2S microphone
- MAX98357A I2S amplifier
- 4Ω speaker
- 3 physical switches
- Power banks
- Capacitor
- Jumper wires and breadboards

## Project Objectives
- Implement real-time wireless voice communication.
- Compare different digital encoding techniques.
- Study the effect of encoding on reconstructed voice quality.
- Evaluate communication performance.

## Technologies
- Arduino IDE
- ESP32
- ESP-NOW
- I2S
- Embedded C/C++

## Project Status
Hardware prototype and software implementation.
