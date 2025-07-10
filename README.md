# OpenDatalogger Thermometer
This repository contains the hardware (schematics and PCB layout) and software for a low-cost data-logging thermometer

Powered by the Raspberry Pi Pico and it's PIO peripheral, the board is capable of supporting up to 8 digital pulse train temperature sensors, such as the LMT01. The Pico's PIO peripheral can measure all 8 sensors simultaneously. The LMT01 is capable of outputting about 10 samples per second, and this board can achieve this theoretical maximum for all 8 sensors, for a total of 80 samples per second if all 8 channels are populated. The Pico is capable of much higher data rates if faster sensors are used.
The board contains a comparator (with optional hysteresis) for each channel to render a crisp digital signal for the PIO. The Pico can then log the values to any combination of:

1. A 20x4 character display,

2. The Pico's serial over USB, and

3. An SD card.

The board can either be powered over the Pico's built-in micro-USB connector or a dedicated USB-C connector for power.

The PCB is designed to meet JLCPCB / PCBway's special $2/$5 + shipping price. Component cost for the base station is about $50NZD, and the sensor components cost about $7NZD per channel. In total the full 8-channel configuration costs about $100NZD + shipping.
