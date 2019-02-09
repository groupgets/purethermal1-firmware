# PureThermal 1/2/mini Reference Firmware

[![Build Status](https://travis-ci.org/groupgets/purethermal1-firmware.svg?branch=master)](https://travis-ci.org/groupgets/GetThermal)

The [PureThermal 1](https://groupgets.com/manufacturers/groupgets-labs/products/pure-thermal-1-flir-lepton-dev-kit)
is an embedded development platform for the FLIR Lepton thermal imager, created by
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


### Clone and Build

    git clone https://github.com/groupgets/purethermal1-firmware
    cd purethermal1-firmware

If you wish, you can modify `Src/project_config.h` to customize your build. Then you can simply build:

    make

or

    make USART_DEBUG=1

alternatively, you can enable semihosting support to printf over JTAG/SWD:

    make GDB_SEMIHOSTING=1

See the "Debugging" section below for more information.

If you have [OpenOCD](http://openocd.org) installed (available in most package management systems), you can then flash
the compiled firmware (see next section for required hardware):

    make flash

## Installing Release Firmware

**Required Hardware**

* [STLInk/V2]()
* [ARM MCU JTAG Adapter for PureThermal 1]( https://groupgets.com/manufacturers/getlab/products/arm-mcu-jtag-adapter-for-purethermal-1)
* [PureThermal 1 - FLIR Lepton Smart I/O Module]( https://groupgets.com/manufacturers/getlab/products/purethermal-1-flir-lepton-smart-i-o-module)
* Micro USB Cable

**Hardware Instructions**

1. Attach ribbon cable from STLINK/V2 into ARM Adapter.
2. Plug in ribbon cable from ARM Adapter into PureThermal1's ZIF socket.
3. Insert mini-usb cable into STLINK/V2 and computer.
4. Insert micro-usb cable into PureThermal 1 and computer.
5. Power on PureThermal 1 if not already on with ON/OFF button press.

### Mac OS

  * [Video Guide](https://www.youtube.com/watch?v=omKEBaMih5g)

1. In terminal download and install the STLink software using homebrew:

        brew install stlink

2. Navigate to the folder which contains the PureThermal 1 firmware (pt1-vx.xx.x.bin or pt1-vx.xx.x+Y16), or build with the instructions above.
3. Use the following command to erase the firmware on the PureThermal 1:

        st-flash erase

4. To write the firmware use one of the following commands (replace the X's with the version of firmware you have downloaded):

        st-flash --reset write pt1-vX.XX.X.bin 0x08000000

    Y16 Firmware:

        st-flash --reset write pt1-vX.XX.X+Y16.bin 0x08000000

    Output of a Makefile build:

        st-flash --reset write main.bin 0x08000000

### Linux

* See the [texane/stlink github page](https://github.com/texane/stlink) for detailed instructions for various Linux distributions. 

### Windows

  * [Video Guide](https://www.youtube.com/watch?v=epo64Zp7TsA)

1. [Download the STM32 ST-LINK Utility](http://www.st.com/en/embedded-software/stsw-link004.html) (STSW-LINK004)
2. Install STM32 ST-Link Utility and then open the software.
3. Select "Target" then "Connect" to connect the software to the PureThermal 1.
4. Next select "File" then "Open file...", now navigate to the folder which contains the PureThermal 1 firmware and select & open the firmware you want to flash (pt1-vX.XX.X.bin or pt1-vX.XX.X+Y16).
5. Finally click "Target" then "Program & Verify...", make sure that the "Reset after programming" box is selected then press the "Start" button.

### Installing firmware via DFU-Mode (for PT2 boards and really old PT1 boards)

Make sure you use at least 1.1.0 becuase earlier versions didn't support the PT1 and PT2 board from the same binary. 

If you're not compiling from source you can download the firmware from [https://github.com/groupgets/purethermal1-firmware/releases](https://github.com/groupgets/purethermal1-firmware/releases) and substitute the .bin file in the tgz for main.bin in the commands below.

#### Entering bootloader (DFU) mode


1. Locate the buttons to press on your PureThermal board.

    ![PT2 Buttons](images/PT2DfuButtons.jpg)
2. Press and hold the BOOT button
3. Without releasing BOOT, press and release RST
4. Release BOOT
5. When you successfully enter DFU mode, the indicator LED will stop blinking and dim to half brightness

#### Linux / MacOS

Install dfu-util:

    sudo apt-get install dfu-util

or 

    brew install dfu-util

or your system's package manager. 

Then run:

    dfu-util -a 0 -D main.bin -s 0x08000000

or use:

    scripts/flash.sh

#### Windows

DfuSe USB drivers. 
http://www.st.com/web/en/catalog/tools/FM147/CL1794/SC961/SS1533/PF257916#

win32 DFU tools
https://files.groupgets.com/purethermal/win32_dfu.zip

To install DFU drivers, may need to use the device manager to select the st drivers

extract `win32_dfu.zip` to the current folder.

    win32_dfu\bin2dfu --i main.bin --a 0x08000000 --o main.dfu
    win32_dfu\DfuSeCommand -c -d --fn main.dfu

or use:

    scripts/make_and_flash.bat



## Debugging

This section covers `printf` over JTAG/SWD though GDB semihosting. You can also set breakpoints using GDB+OpenOCD,
or with using the provided Eclipse projects. Additionally, you can enable printf support over UART.

### Makefile projects

For Makefile projects, you can enable semihosting support to enable printf over JTAG/SWD. This works with STLink/V2,
Segger JLink, and should work with others as well.

You must build the code with semihosting support enabled (GDB_SEMIHOSTING=1). Note that if you get build errors,
you might have to `make clean` and retry. Simply run the following:

    GDB_SEMIHOSTING=1 make debug

The debugger should remain attached, and you should see printf output on the console, like so:

    ...
    xPSR: 0x01000000 pc: 0x08010b8c msp: 0x20020000
    semihosting is enabled
    auto erase enabled
    Info : device id = 0x10006431
    Info : flash size = 512kbytes
    wrote 131072 bytes from file main.bin in 4.133242s (30.968 KiB/s)
    adapter speed: 2000 kHz
    Info : JTAG tap: stm32f4x.cpu tap/device found: 0x4ba00477 (mfg: 0x23b (ARM Ltd.), part: 0xba00, ver: 0x4)
    Info : JTAG tap: stm32f4x.bs tap/device found: 0x06431041 (mfg: 0x020 (STMicroelectronics), part: 0x6431, ver: 0x0)
    Hello, Lepton!
    reading_tmp007_regs...
    Initialized...

### Eclipse projects

Semihosting also works in Eclipse/OpenSTM32 projects. To enable this, locate the target debug configuration for
the project and add the following line to Initialization Commands in the Startup tab:

    monitor arm semihosting enable

You should see print statements appear in the debugger console view.

## For More Information

Refer to [PureThermal 1 development wiki](https://github.com/groupgets/purethermal1-firmware/wiki).


## Licensing and Support

This firmware is licensed under The MIT License (MIT), see `LICENSE` file for details.

Inquiries for support should be directed at this project's maintainers, GroupGets, LLC (GroupGets), and not FLIR
Systems, Inc. (FLIR). Additional FLIR-provided source code is being released by GroupGets to this project under the
express written consent of FLIR, and covered by the project's license, with the expectation that no warranties
or support of any kind will be provided by FLIR.
