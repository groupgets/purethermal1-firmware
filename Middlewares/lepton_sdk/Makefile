# Compile the Lepton SDK for STM32
export

CFLAGS := $(patsubst -I%,-I../%,$(CFLAGS))

LIB_OUT = libleptonsdk.a

SRCS := $(shell find . -name *.[cs])

LIB_OBJS = $(sort $(patsubst %.c,%.o,$(SRCS))) 

.PHONY: all
all: $(LIB_OUT)

$(LIB_OUT): $(LIB_OBJS)
	$(AR) $(ARFLAGS) $@ $(LIB_OBJS)

.PHONY: clean
clean:
	-rm -f $(LIB_OBJS) $(LIB_OUT)

ifdef NOOP
# Make command to use for dependencies
MAKE=make
RM=rm
MKDIR=mkdir
CP=cp

# -----Begin user-editable area-----

# -----End user-editable area-----

#
# Configuration: Release
#
OUTDIR=.
OUTFILE=$(OUTDIR)/LEPTON_SDK.a
CFG_INC=
CFG_LIB=
CFG_OBJ=
COMMON_OBJ=$(OUTDIR)/crc16fast.o \
	$(OUTDIR)/LEPTON_AGC.o $(OUTDIR)/LEPTON_VID.o \
	$(OUTDIR)/LEPTON_I2C_Protocol.o $(OUTDIR)/LEPTON_I2C_Service.o \
	$(OUTDIR)/LEPTON_SDK.o $(OUTDIR)/LEPTON_SYS.o
OBJ=$(COMMON_OBJ) $(CFG_OBJ)
ALL_OBJ=$(OUTDIR)/crc16fast.o \
	$(OUTDIR)/LEPTON_AGC.o $(OUTDIR)/LEPTON_VID.o \
	$(OUTDIR)/LEPTON_I2C_Protocol.o $(OUTDIR)/LEPTON_I2C_Service.o \
	$(OUTDIR)/LEPTON_SDK.o $(OUTDIR)/LEPTON_SYS.o

COMPILE=g++ -fpermissive -mno-cygwin -c  -v  -o "$(OUTDIR)/$(*F).o" $(CFG_INC) "$<"
LINK=ar -rs  "$(OUTFILE)" $(OBJ)

# Pattern rules
$(OUTDIR)/%.o : %.c
	$(COMPILE)

# Build rules
all: $(OUTFILE)

$(OUTFILE): $(OUTDIR)  $(OBJ)
	$(LINK)

$(OUTDIR):
	$(MKDIR) -p "$(OUTDIR)"


# Rebuild this project
rebuild: cleanall all

# Clean this project
clean:
	$(RM) -f $(OUTFILE)
	$(RM) -f $(OBJ)

# Clean this project and all dependencies
cleanall: clean
endif
