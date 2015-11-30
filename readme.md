# Pure Thermal 1 Reference Firmware

The Pure Thermal 1 is an embedded development platform for the FLIR Lepton thermal imager, created by
GroupGets Labs. It is based around the STM32F4, a powerful ARM Cortex-M MCU by ST Microelectronics, and its
various IO capabilities and open source firmware to make it easy integrate a FLIR Lepton into any environment.


## Purpose of This Project

It is intended that you use this firmware as a starting point for you own applications. For many applications
where you wish to process thermal image data on a host PC, you can probably use pre-built binaries of this firmware
and use as-is. For others, forking this project and making your own modifications to this firmware is encouraged.


## Building the Firmware

This is a Makefile-based project, and we use the GCC ARM Embedded toolchain from https://launchpad.net/gcc-arm-embedded,
however, other toolchains that work for the STM32 (such as Mentor's Sourcery toolchain) should also work.


### Compiler Installation

The following are compiler installation instructions for various platforms, required for building the firmware.


#### Ubuntu 10.04/12.04/14.04/14.10 32/64-bit

    sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
    sudo apt-get update
    sudo apt-get install gcc-arm-none-eabi


#### OS X

TODO


### Windows

TODO


### Clone and Build

    git clone git@bitbucket.org:groupgets/lepton-devboard.git
    cd lepton-devboard
    make


## Installing the Firmware

TODO


## For More Information

Refer to project wiki


## License

TODO
