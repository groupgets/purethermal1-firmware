# Make all libraries
export

CFLAGS := $(patsubst -I%,-I../%,$(CFLAGS))

SUBDIRS := ST \
           lepton_sdk

.PHONY: subdirs $(SUBDIRS)
.PHONY: clean

subdirs: ${SUBDIRS}

${SUBDIRS}:
	${MAKE} -C $@

clean:
	-for d in $(SUBDIRS); do (echo "cleaning up in $$d"; cd $$d; $(MAKE) clean ); done
