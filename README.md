# DAC/ADC Dual Core Firmware

[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Firmware for Young Lab AD5791 DACs / AD7734 ADCs. New features include precise timings (<300ns error) for SPI comms, new buffer ramp options, and data acquisition being in-tandem with transmission to LabRAD.

The firmware is designed to be easily extensible, as new peripherals can simply be added with little consideration for how other peripherals are written, as operations are all added independently to a centralized registry.

**If you have any questions**, feel free to email (or slack) [markzakharyan@ucsb.edu](mailto:markzakharyan@ucsb.edu)

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [License](#license)

## Installation

This firmware is comprised of two individual PlatformIO projects, one for the M7 core and the other for the M4. To build and upload to the Arduino Giga, you need to install PlatformIO and separately upload the firmware for **both** the M7 and M4 processors (found in their respective folders). There is no extra configuration you need to do, as all processor information is contained in each project's platformio.ini file.

## Usage

Since PlatformIO does not yet have support for the Arduino Giga, we have forked a custom implementation of PlatformIO for the Giga found at [https://github.com/afylab/platform-ststm32#develop](https://github.com/afylab/platform-ststm32#develop). This is what each project uses to build for the Giga, and is found in each project's platformio.ini.

A full guide on the code structure is being developed and this section will be periodically updated.


## License

This project is licensed under the [MIT License](LICENSE).
