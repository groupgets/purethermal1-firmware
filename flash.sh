#!/bin/sh

dfu-util -a 0 -D main.bin -s 0x08000000
