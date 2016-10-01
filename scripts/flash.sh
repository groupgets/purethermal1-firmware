#!/bin/sh

dfu-util -a 0 -d 0483:df11 -D main.bin -s 0x08000000
