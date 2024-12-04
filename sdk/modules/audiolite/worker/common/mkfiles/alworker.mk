ifeq ($(BUILD_EXECUTE),1)

all: depend $(SPK)

# For alworker common dir
ALWORKER_COMMONMKS = $(ALWORKER_COMMON)/mkfiles

CSRCS += alworker_comm.c
CSRCS += alworker_commfw.c
CSRCS += alworker_memblk.c

# For SPK tools

MKSPK_TOOLDIR = $(TOPDIR)/tools/cxd56
MKSPK_MKFILE = $(MKSPK_TOOLDIR)/Makefile.host
MKSPK_TOOL = $(MKSPK_TOOLDIR)/mkspk
MKSPK_TOOL_OPT = -c 2

# Setup to build and linking worker support library

WORKER_DIR = $(SDKDIR)$(DELIM)modules$(DELIM)asmp$(DELIM)worker
WORKER_LIB = $(WORKER_DIR)$(DELIM)libasmpw$(LIBEXT)
LDLIBPATH += -L $(WORKER_DIR)
LDLIBS += -lasmpw

ifneq ($(CONFIG_ASMP_WORKER_CMSIS),)
include $(ALWORKER_COMMONMKS)/cmsis.mk
endif

ifeq ($(ALWORKER_USE_RESAMPLER),1)
include $(ALWORKER_COMMONMKS)/resampler.mk
endif

AUDIOLITE_DIR = $(ALWORKER_COMMON)/../..

CFLAGS += -DBUILD_TGT_ASMPWORKER

VPATH = $(ALWORKER_COMMON) $(VPATH_DIRS)

DELTGT_FILES = $(CSRCS:.c=.o)

INCDIRS += . $(ALWORKER_COMMON) $(AUDIOLITE_DIR)/../include

CFLAGS += -O3
ifeq ($(WINTOOL),y)
CFLAGS += -I"$(shell cygpath -w $(SDKDIR)/modules/asmp/worker)"
CFLAGS += $(foreach d,$(INCDIRS), -I"$(shell cygpath -w $(d))")
else
CFLAGS += -I$(SDKDIR)/modules/asmp/worker
CFLAGS += $(foreach d,$(INCDIRS), -I$(d))
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

LIBGCC = "$(shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name)"

$(COBJS): %$(OBJEXT): %.c
	@echo "CC: $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(AOBJS): %$(OBJEXT): %.S
	@echo "AS: $<"
	$(Q) $(CC) -c $(AFLAGS) $< -o $@

$(BIN): $(COBJS) $(AOBJS) | $(WORKER_LIB) $(CMSIS_DSP_LIB)
	@echo "LD: $@"
	$(Q) $(LD) $(LDRAWELFFLAGS) -e main --gc-sections $(LDLIBPATH) -o $@ \
		--start-group $(ARCHCRT0OBJ) $^ $(LDLIBS) $(LIBGCC) --end-group
	$(Q) cp $(BIN) $(BIN).debug.elf
	$(Q) $(STRIP) -d $(BIN)

$(MKSPK_TOOL):
	@$(MAKE) -C $(MKSPK_TOOLDIR) -f $(MKSPK_MKFILE) TOPDIR="$(TOPDIR)"

$(SPK): $(MKSPK_TOOL) $(BIN)
	@echo "MKSPK: $@"
	$(MKSPK_TOOL) $(MKSPK_TOOL_OPT) $(BIN) $(BIN) $(SPK)
	mkdir -p $(SDKDIR)/workerspks
	cp $(SPK) $(SDKDIR)/workerspks

clean:
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(BIN).debug.elf)
	$(call DELFILE, $(DELTGT_FILES))
	$(call DELFILE, $(SPK))
	$(call DELFILE, $(SDKDIR)/workerspks/$(SPK))
	$(call CLEAN)

ROOTDEPPATH  = --dep-path . $(foreach d,$(INCDIRS), --dep-path $(d))

.depend:
	@$(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(CSRCS) >Make.dep
	@touch $@

depend: .depend

# Rules for building worker support library

libdepend:
	$(Q) $(MAKE) -C $(WORKER_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" depend

$(WORKER_LIB): libdepend
	$(Q) $(MAKE) -C $(WORKER_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libasmpw$(LIBEXT)

context:

-include Make.dep

else # ($(BUILD_EXECUTE),1)

clean:

distclean:

endif # ($(BUILD_EXECUTE),1)
