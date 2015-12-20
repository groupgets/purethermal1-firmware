make
win32_dfu\bin2dfu --i main.bin --a 0x08000000 --o main.dfu
win32_dfu\DfuSeCommand -c -d --fn main.dfu
pause
