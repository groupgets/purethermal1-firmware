# Make all libraries
export

CFLAGS := $(patsubst -I%,-I../../%,$(CFLAGS))

SUBDIRS := $(shell ls -F|grep /|grep -v CVS)

.PHONY: subdirs $(SUBDIRS)
.PHONY: clean

subdirs: ${SUBDIRS}

${SUBDIRS}:
	${MAKE} -C $@

clean:
	-for d in $(SUBDIRS); do (echo "cleaning up in $$d"; cd $$d; $(MAKE) clean ); done
