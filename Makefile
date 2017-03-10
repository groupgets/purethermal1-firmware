export

SYSTEM  ?= arm-none-eabi-
CC      = $(SYSTEM)gcc
CCDEP   = $(SYSTEM)gcc
LD      = $(SYSTEM)gcc
AR      = $(SYSTEM)ar
AS      = $(SYSTEM)gcc
OBJCOPY = $(SYSTEM)objcopy
OBJDUMP	= $(SYSTEM)objdump
GDB     = $(SYSTEM)gdb
SIZE    = $(SYSTEM)size

OCD ?= openocd

CPU = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16

DEVICE_FAMILY = STM32F4xx
STARTUP_FILE = stm32f411xe
DEVICE_TYPE = STM32F411xE
SYSTEM_FILE = stm32f4xx

CMSIS = Drivers/CMSIS
CMSIS_DEVSUP = $(CMSIS)/Device/ST/$(DEVICE_FAMILY)/
CMSIS_OPT = -D$(DEVICE_TYPE) -DUSE_HAL_DRIVER
OTHER_OPT = "-D__weak=__attribute__((weak))" "-D__packed=__attribute__((__packed__))" 

LDSCRIPT = ./STM32F411CEUx_FLASH.ld

SRCDIR := Src/
INCDIR := Inc/

LIBDIR := Drivers/
MDLDIR := Middlewares/

#LIBINC := $(shell find $(INCDIR) -name *.h -printf "-I%h/\n"|sort|uniq)
#LIBINC += $(shell find $(LIBDIR) -name *.h -printf "-I%h/\n"|sort|uniq)
#LIBINC += $(shell find $(MDLDIR) -name *.h -printf "-I%h/\n"|sort|uniq|sed -e 's/lwip\///'|sed -e 's/arch\///')

LIBINC := -IInc
LIBINC += -IDrivers/STM32F4xx_HAL_Driver/Inc
LIBINC += -IMiddlewares/ST/STM32_USB_Device_Library/Core/Inc
LIBINC += -IMiddlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc
LIBINC += -IMiddlewares/ST/STM32_USB_Device_Library/Class/video/Inc
LIBINC += -IDrivers/CMSIS/Include
LIBINC += -IDrivers/CMSIS/Device/ST/STM32F4xx/Include
LIBINC += -IMiddlewares/lepton_sdk/Inc


LIBS := ./$(LIBDIR)/STM32F4xx_HAL_Driver/libstm32fw.a
LIBS += ./$(MDLDIR)/ST/STM32_USB_Device_Library/libstm32usbdev.a
LIBS += ./$(MDLDIR)/lepton_sdk/libleptonsdk.a
LIBS += -lm
  
# INCLUDES = -I$(SRCDIR) $(LIBINC)
INCLUDES = $(LIBINC)
CFLAGS  = $(CPU) $(CMSIS_OPT) $(OTHER_OPT) -Wall -fno-common -fno-short-enums -Os $(INCLUDES) -Wfatal-errors -std=c99 -DGIT_VERSION
ifdef GDB_SEMIHOSTING
	CFLAGS += -DGDB_SEMIHOSTING
endif
ifdef USART_DEBUG
	USART_DEBUG_SPEED ?= 115200
	CFLAGS += -DUSART_DEBUG -DUSART_DEBUG_SPEED=$(USART_DEBUG_SPEED)
endif
ASFLAGS = $(CFLAGS) -x assembler-with-cpp
LDFLAGS = -Wl,--gc-sections,-Map=$*.map,-cref -fno-short-enums -Wl,--no-enum-size-warning -T $(LDSCRIPT) $(CPU)
ifdef GDB_SEMIHOSTING
	LDFLAGS += -specs=nano.specs -specs=rdimon.specs -lrdimon
else
	LDFLAGS += -specs=nosys.specs
endif
ARFLAGS = cr
OBJCOPYFLAGS = -Obinary
OBJDUMPFLAGS = -S

STARTUP_OBJ = $(CMSIS_DEVSUP)/Source/Templates/gcc/startup_$(STARTUP_FILE).o
SYSTEM_OBJ = $(CMSIS_DEVSUP)/Source/Templates/system_$(SYSTEM_FILE).o

.PHONY: print_vars

BIN = main.bin

OBJS = $(sort \
 $(patsubst %.c,%.o,$(wildcard Src/*.c)) \
 $(patsubst %.s,%.o,$(wildcard Src/*.s)) \
 $(STARTUP_OBJ) \
 $(SYSTEM_OBJ))

all: $(BIN) print_vars

reset:
	$(OCD) -c init -c "reset run" -c shutdown
	#$(GDB) main.out <reset.gdb

flash: $(BIN)
	$(OCD) -f PureThermal1.cfg \
	       -c "init" \
	       -c "reset halt" \
	       -c "flash write_image erase $(BIN) 0x08000000" \
	       -c "reset run" \
	       -c "shutdown" \
	       || echo "ERROR: could not connect to target, please try again"

debug: $(BIN)
	$(OCD) -f PureThermal1.cfg \
	       -c "init" \
	       -c "reset halt" \
	       -c "arm semihosting enable" \
	       -c "flash write_image erase $(BIN) 0x08000000" \
	       -c "reset run" \
	       || echo "ERROR: could not connect to target, please try again"

$(BIN): main.out
	$(OBJCOPY) $(OBJCOPYFLAGS) main.out $(BIN)
	$(OBJCOPY) -O ihex main.out main.hex
	$(OBJDUMP) $(OBJDUMPFLAGS) main.out > main.list
	$(SIZE) main.out
	@echo Make finished

Inc/version.h: .git/HEAD .git/index
	echo "#ifndef VERSION_H" > $@
	echo "#define VERSION_H" >> $@
	echo "#define BUILD_GIT_SHA \"$(shell git describe --tags)\"" >> $@
	echo "#define BUILD_DATE \"$(shell date "+%Y-%m-%d %H:%M:%S")\"" >> $@
	echo "#endif" >> $@

# TARGET = main
# 
# $(BIN): $(OBJS)
# 	$(CC) -o $(TARGET).elf $(LDFLAGS) $(OBJS)	$(LDLIBS)
# 	$(OBJCOPY) -O ihex   $(TARGET).elf $(TARGET).hex
# 	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

main.out: $(LIBS) $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

$(LIBS): libs

libs:
	@$(MAKE) -C $(LIBDIR)
	@$(MAKE) -C $(MDLDIR)

libclean: clean
	@$(MAKE) -C $(LIBDIR) clean
	@$(MAKE) -C $(MDLDIR) clean

clean:
	@$(MAKE) -C $(LIBDIR) clean
	@$(MAKE) -C $(MDLDIR) clean
	-rm Inc/version.h
	-rm -f $(OBJS)
	-rm -f main.list main.out main.hex main.map main.bin .depend

depend dep: .depend

include .depend

.depend: Src/*.c | Inc/version.h
	$(CCDEP) $(CFLAGS) -MM $^ | sed -e 's@.*.o:@Src/&@' > .depend 

.c.o:
	@echo cc $<
	@$(CC) $(CFLAGS) -c -o $@ $<

.s.o:
	@echo as $<
	@$(AS) $(ASFLAGS) -c -o $@ $<

print_vars:
ifdef USART_DEBUG
	$(info *** Using USART for print debugging ***)
	$(info USART_DEBUG_SPEED=$(USART_DEBUG_SPEED))
endif
