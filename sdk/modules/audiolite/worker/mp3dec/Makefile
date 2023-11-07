############################################################################
# sdk/modules/audiolite/worker/mp3dec/Makefile
#
#   Copyright 2023 Sony Semiconductor Solutions Corporation
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name of Sony Semiconductor Solutions Corporation nor
#    the names of its contributors may be used to endorse or promote
#    products derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

include $(APPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

ifeq ($(CONFIG_AUDIO_LITE_MP3DEC_SUBCORE_ELF),y)
BUILD_EXECUTE = 1
else ifeq ($(FORCE_BUILD),1)
BUILD_EXECUTE = 1
else
BUILD_EXECUTE = 0
endif

ifeq ($(BUILD_EXECUTE),1)

BIN = mp3dec
all: depend $(BIN)

# Setup to build and linking worker support library

WORKER_DIR = $(SDKDIR)$(DELIM)modules$(DELIM)asmp$(DELIM)worker
WORKER_LIB = $(WORKER_DIR)$(DELIM)libasmpw$(LIBEXT)
LDLIBPATH += -L $(WORKER_DIR)
LDLIBS += -lasmpw

CMSISDIR = $(SDKDIR)$(DELIM)..$(DELIM)externals$(DELIM)cmsis
CMSIS_DSP_DIR = $(CMSISDIR)$(DELIM)dsp

# Setup to build and linking CMSIS DSP library

include $(CMSISDIR)/LibIncludes.mk

CMSIS_DSP_LIB = $(CMSIS_DSP_DIR)$(DELIM)libarm_cortexM4lf_math$(LIBEXT)
LDLIBPATH += -L $(CMSIS_DSP_DIR)
LDLIBS += -larm_cortexM4lf_math

AUDIOLITE_DIR = ../..
COMMON_DIR    = ../common
MINIMP3_DIR   = ../ext_libs/minimp3
SPEEXDSP_DIR  = ../ext_libs/speexdsp_resample

CSRCS =  entry.c mp3dec_main.c
CSRCS += sprmp3_msghandler.c
CSRCS += sprmp3_sendback.c
CSRCS += alworker_comm.c
CSRCS += minimp3.c
CSRCS += minimp3_ex.c
CSRCS += resample.c
CFLAGS += -DUSE_CMSIS -DBUILD_TGT_ASMPWORKER

VPATH = $(COMMON_DIR) $(MINIMP3_DIR) $(SPEEXDSP_DIR)

DELTGT_FILES = $(CSRCS:.c=.o)

INCDIRS = $(COMMON_DIR) $(MINIMP3_DIR) $(SPEEXDSP_DIR) $(AUDIOLITE_DIR)/../include

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

clean:
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(BIN).debug.elf)
	$(call DELFILE, $(DELTGT_FILES))
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

$(CMSIS_DSP_LIB):
	$(Q) $(MAKE) -C $(CMSIS_DSP_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)"

context:

-include Make.dep

else # ($(BUILD_EXECUTE),1)

clean:

distclean:

endif # ($(BUILD_EXECUTE),1)
