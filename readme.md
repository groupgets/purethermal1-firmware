# Pure Thermal 1 Reference Firmware

The Pure Thermal 1 is an embedded development platform for the FLIR Lepton thermal imager, created by
GroupGets Labs. It is based around the STM32F4, a powerful ARM Cortex-M MCU by ST Microelectronics, and its
various IO capabilities and open source firmware to make it easy integrate a FLIR Lepton into any environment.


## Purpose of This Project

It is intended that you use this firmware as a starting point for you own applications. For many applications
where you wish to process thermal image data on a host PC, you can probably use pre-built binaries of this firmware
and use as-is. For others, forking this project and making your own modifications to this firmware is encouraged.


## Building the Firmware

This is a Makefile-based project, and we use the GCC ARM Embedded toolchain from https://launchpad.net/gcc-arm-embedded/+download,
however, other toolchains that work for the STM32 (such as Mentor's Sourcery toolchain) should also work.


### Compiler Installation

The following are compiler installation instructions for various platforms, required for building the firmware.


#### Ubuntu 10.04/12.04/14.04/14.10 32/64-bit

    sudo add-apt-repository ppa:terry.guo/gcc-arm-embedded
    sudo apt-get update
    sudo apt-get install gcc-arm-none-eabi


#### OS X

Launchpad has toolchain binaries for OS X. To keep things simple, we'll install then into `/usr/local/gcc-arm-none-eabi-*`

    sudo mkdir -p /usr/local
    curl -L https://launchpad.net/gcc-arm-embedded/4.9/4.9-2015-q3-update/+download/gcc-arm-none-eabi-4_9-2015q3-20150921-mac.tar.bz2 -o ~/Downloads/gcc-arm-embedded.tar.bz2
    sudo tar xjf ~/Downloads/gcc-arm-embedded.tar.bz2 -C /usr/local && rm ~/Downloads/gcc-arm-embedded.tar.bz2

Then you need to add this location to your path. Add to your .bashrc, or every time you open a new shell and wish to build execute:

    export PATH=$PATH:/usr/local/gcc-arm-none-eabi-4_9-2015q3/bin


#### Windows

Toolchain
https://launchpad.net/gcc-arm-embedded

Msys
http://www.mingw.org/wiki/msys 
-or- install MINGW with MSYS. 
http://sourceforge.net/projects/mingw/files/latest/download?source=files

Command Line directions for building and updating the firmware:
Install Above software.
Add to your path, MSYS `/bin` and Toolchain arm gcc `/bin` folders. 


### Clone and Build

    git clone https://github.com/groupgets/purethermal1-firmware
    cd purethermal1-firmware

If you wish, you can modify `Src/project_config.h` to customize your build. Then you can simply build:

    make

or

    make USART_DEBUG=1


## Installing the Firmware

To put the device in boot loader mode:
Plug in USB Cable
Power on device first with ON/OFF button press
While holding the BOOT0 button, press the RST button, Release the RST first, then release the BOOT0 button

### Linux

Install dfu-util:

    sudo apt-get install dfu-util

Then run:

    dfu-util -a 0 -D main.bin -s 0x08000000

or use:

    scripts/flash.sh


### OS X

Install dfu-util via Homebrew

    brew install dfu-util

Then run:

    dfu-util -a 0 -D main.bin -s 0x08000000

or use:

    scripts/flash.sh


### Windows

DfuSe USB drivers. 
http://www.st.com/web/en/catalog/tools/FM147/CL1794/SC961/SS1533/PF257916#

win32 DFU tools
https://files.groupgets.com/purethermal/win32_dfu.zip

To unstall DFU drivers, may need to use the device manager to select the st drivers

extract `win32_dfu.zip` to the current folder.

    win32_dfu\bin2dfu --i main.bin --a 0x08000000 --o main.dfu
    win32_dfu\DfuSeCommand -c -d --fn main.dfu

or use:

    scripts/make_and_flash.bat

Also, see the document [GroupGets PureThermal 1 Firmware Installation Guide for Windows OS](https://files.groupgets.com/purethermal/GroupGets_Pure_Thermal_Firmware_Installation_Guide_for_Windows_OS.pdf)


## For More Information

Refer to [PureThermal 1 development wiki](https://github.com/groupgets/purethermal1-firmware/wiki).


## License

This firmware is licensed under The MIT License (MIT), see `LICENSE` file for details.
