# Robot Remote Control PlatformIO Project

## Project Overview

This project implements a three-part control system for a small tracked robot built around  ATmega328P microcontrollers and the nRF24L01+ radio module family. The system is split into a handheld RF transmitter, a robot-side RF receiver with motor control and autonomous IR tracking logic, and a separate IR beacon transmitter that generates a 38 kHz carrier in repeated bursts.

The codebase is organized as a single PlatformIO project with separate firmware entrypoints for each microcontroller. This makes it possible to build and upload the controller, the robot receiver, and the IR transmitter independently while keeping the radio protocol and overall project structure in one repository.

## Current Functional State

At the current stage, the project is intended to support the following behavior:
- manual radio control of a tracked robot platform
- switching between manual and autonomous operation
- IR beacon reception using three directional sensors
- direction estimation based on left / center / right IR activity
- autonomous correction of movement toward the IR beacon
- autonomous operation with binary ON/OFF track drive control

## System Architecture

The robot control stack is composed of:

- a transmitter / handheld controller based on an ATmega328P or other Arduino-compatible MCU
- a robot-side receiver based on Arduino Nano
- a differential-drive tracked platform
- a separate IR beacon transmitter
- manual and autonomous operating modes

## Firmware Variants

This repository currently contains three firmware variants:

- `src/tx_main.cpp`: RF handheld controller firmware
- `src/rx_main.cpp`: robot-side RF receiver, motor controller, and IR-guided autonomous logic
- `src/ir_main.cpp`: standalone 38 kHz IR beacon transmitter

## Functional Description

### Manual Mode

In manual mode, the robot is controlled remotely over the nRF24L01+ radio link.

The controller uses a button-based interface rather than an analog joystick. The available controls are:

- left track forward
- left track reverse
- right track forward
- right track reverse
- MANUAL/AUTO mode toggle
- action button `A1`
- action button `A2`

The left and right tracks are controlled independently, which allows:

- forward motion
- reverse motion
- in-place rotation
- turning by asymmetric track control

At the radio protocol level, the transmitter sends:

- the current operating mode: `MANUAL` or `AUTO`
- a bitmask of the currently pressed buttons

### Autonomous Mode

In `AUTO` mode, the robot ignores manual drive commands and relies on local IR beacon tracking logic running on the receiver MCU.

The robot uses three front-mounted IR receivers:

- left
- center
- right

Based on the number of detected IR bursts inside a time window, the robot estimates the direction of the IR source and chooses a movement decision such as:

- rotate left
- rotate right
- move forward
- search for the signal after losing contact

The autonomous guidance logic is based on:

- comparing `LEFT`, `CENTER`, and `RIGHT` sensor activity
- remembering the last direction from which the beacon was detected
- continuously correcting the motion direction while tracking the beacon


## Diagnostics and Debugging

Both the transmitter and receiver support basic diagnostic commands over UART / Serial:

- `DEBUG ON`
- `DEBUG OFF`
- `DEBUG`
- `STATUS`
- `HELP`

These commands can be used to inspect:

- the current operating mode
- the button state seen by the transmitter
- radio communication behavior
- IR sensor activity
- autonomous tracking decisions

## Radio Protocol

The shared radio protocol is defined in `include/protocol.h`.

Current radio settings:

- address: `CTRL1`
- channel: `76`
- data rate: `RF24_250KBPS`
- PA level: `RF24_PA_MIN`

Packet format:

```cpp
struct Packet {
  uint8_t mode;
  uint8_t buttons_bitmask;
};
```

### `mode`

- `0` = `MODE_MANUAL`
- `1` = `MODE_AUTO`

### `buttons_bitmask`

- bit 0 = `LEFT_FWD`
- bit 1 = `LEFT_REV`
- bit 2 = `RIGHT_FWD`
- bit 3 = `RIGHT_REV`
- bit 4 = `A1`
- bit 5 = `A2`

## Pinout

### Transmitter (`tx_main.cpp`)

Buttons:

- `D2` = `MODE`
- `D3` = `A1`
- `D4` = `A2`
- `D5` = `LEFT_REV`
- `D6` = `RIGHT_REV`
- `D7` = `RIGHT_FWD`
- `D8` = `LEFT_FWD`

Each button is wired as `pin -> button -> GND`, with `INPUT_PULLUP` enabled in firmware.

nRF24L01+:

- `D9` = `CE`
- `D10` = `CSN`
- `D11` = `MOSI`
- `D12` = `MISO`
- `D13` = `SCK`

### Receiver (`rx_main.cpp`)

nRF24L01+:

- `D7` = `CE`
- `D8` = `CSN`
- `D11` = `MOSI`
- `D12` = `MISO`
- `D13` = `SCK`

Motor outputs:

- `D6` = `LEFT_IN1`
- `D5` = `LEFT_IN2`
- `D10` = `RIGHT_IN1`
- `D9` = `RIGHT_IN2`

IR receivers:

- `D2` = `IR_LEFT`
- `D3` = `IR_RIGHT`
- `D4` = `IR_CENTER`

### IR Beacon Transmitter (`ir_main.cpp`)

IR output:

- `D9` = 38 kHz carrier output (`OC1A`)

The IR transmitter firmware configures Timer1 for approximately 38 kHz PWM and alternates between carrier bursts and silent gaps.

## Build and Upload with PlatformIO

The project is configured as a multi-environment PlatformIO workspace. Each environment builds exactly one firmware entrypoint.

Available environments:

- `tx_atmega328p_uart`: transmitter firmware for a standalone ATmega328P with Arduino-compatible UART bootloader
- `rx_nano`: receiver firmware for Arduino Nano
- `ir_atmega328p_uart`: IR beacon transmitter firmware for a standalone ATmega328P with Arduino-compatible UART bootloader

### Build Commands

```bash
pio run -e tx_atmega328p_uart
pio run -e rx_nano
pio run -e ir_atmega328p_uart
```

### Upload Commands

```bash
pio run -e tx_atmega328p_uart -t upload
pio run -e rx_nano -t upload
pio run -e ir_atmega328p_uart -t upload
```

### Serial Monitor

For firmware that exposes UART diagnostics:

```bash
pio device monitor -b 115200
```
