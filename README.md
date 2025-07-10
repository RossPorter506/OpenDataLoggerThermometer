# OpenDatalogger Thermometer
This repository contains the hardware (schematics and PCB layout) and software for a low-cost data-logging thermometer

Powered by the Raspberry Pi Pico and it's PIO peripheral, the board is capable of supporting up to 8 digital pulse train temperature sensors, such as the LMT01. The Pico's PIO peripheral can measure all 8 sensors simultaneously. Currently the supported sampling rates are 1, 2, 4 and 8 Hz. The Pico is capable of much higher data rates if faster sensors are used.
The board contains a comparator (with optional hysteresis) for each channel to render a crisp digital signal for the PIO. The Pico can then log the values to any combination of:

1. A 20x4 character display,

2. The Pico's serial over USB, and

3. An SD card.

The board can either be powered over the Pico's built-in micro-USB connector or a dedicated USB-C connector for power.

The PCB is designed to meet JLCPCB / PCBway's special $2/$5 + shipping price. Component cost for the base station is about $50NZD, and the sensor components cost about $7NZD per channel. In total the full 8-channel configuration costs about $100NZD + shipping.

## Documentation

The PCB files are available in the 'hardware' folder. The schematic and PCB layout were designed using Altium, though these files can be opened by open-source alternatives like KiCAD.
Gerbers are not yet provided.
Schematic files are not yet provided.

Firmware is available in the 'software' folder. The firmware is written in Rust. The firmware can be compiled and flashed to a Pico with `cargo run --release`. See the README in the software folder for more details.

A desktop program to capture the (optional) serial output of the device can be found [here](https://github.com/JackDMatthews/datalogging_thermometer_gui). Development of this GUI is in the early stages, so expect bugs and missing features.

## Known issues

- The power switch does not work correctly on revision 1.0.0. SB2 should be shorted to skip over the malfunctioning Q1. 
The board will remain powered on while a USB cable is plugged in (either the USB-C power header, or the Pico's micro USB connector).

- The opamps listed in the schematic (`TLV9004IPWR`) are too slow to catch the pulses. They can be replaced with `TSV994AIPT`'s, which are pin-compatible.