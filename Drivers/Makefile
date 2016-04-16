# Make all libraries
export

CFLAGS := $(patsubst -I%,-I../../%,$(CFLAGS))

SUBDIRS := CMSIS \
           STM32F4xx_HAL_Driver

.PHONY: subdirs $(SUBDIRS)
.PHONY: clean

subdirs: ${SUBDIRS}

${SUBDIRS}:
	${MAKE} -C $@

clean:
	-for d in $(SUBDIRS); do (echo "cleaning up in $$d"; cd $$d; $(MAKE) clean ); done
