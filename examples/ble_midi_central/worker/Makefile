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

BIN = fm_synth
all: depend $(BIN)

# Setup to build and linking worker support library

WORKER_DIR = $(SDKDIR)$(DELIM)modules$(DELIM)asmp$(DELIM)worker
WORKER_LIB = $(WORKER_DIR)$(DELIM)libasmpw$(LIBEXT)
LDLIBPATH += -L $(WORKER_DIR)
LDLIBS += $(ASMPWORKER_LIB)

CSRCS = fm_synth.c fmsynth.c fmsynth_eg.c fmsynth_op.c alworker_comm.c
CFLAGS += -DBUILD_TGT_ASMPWORKER

VPATH = $(APPDIR)/audioutils/fmsynth ..

DELTGT_FILES = $(CSRCS:.c=.o)

INCDIRS = $(APPDIR)/include

CFLAGS += -O3
CFLAGS += ${INCDIR_PREFIX}$(SDKDIR)/modules/asmp/worker
CFLAGS += ${INCDIR_PREFIX}$(APPDIR)/include

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

$(COBJS): %$(OBJEXT): %.c
	@echo "CC: $<"
	$(Q) $(CC) -c $(CFLAGS) $< -o $@

$(AOBJS): %$(OBJEXT): %.S
	@echo "AS: $<"
	$(Q) $(CC) -c $(AFLAGS) $< -o $@

$(BIN): $(COBJS) $(AOBJS) | $(WORKER_LIB) $(CMSIS_DSP_LIB)
	@echo "LD: $@"
	$(Q) $(LD) $(LDRAWELFFLAGS) -e main --gc-sections $(LDLIBPATH) -o $@ \
		--start-group $(ARCHCRT0OBJ) $^ $(LDLIBS) --end-group
	$(Q) cp $(BIN) $(BIN).debug.elf
	$(Q) $(STRIP) -d $(BIN)

clean:
	$(call DELFILE, $(BIN))
	$(call DELFILE, $(BIN).debug.elf)
	$(call DELFILE, $(DELTGT_FILES))
	$(call CLEAN)

ROOTDEPPATH  = --dep-path . $(foreach d,$(INCDIRS), --dep-path $(d))

depend:

# Rules for building worker support library

libdepend:
	$(Q) $(MAKE) -C $(WORKER_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" depend

$(WORKER_LIB): libdepend
	$(Q) $(MAKE) -C $(WORKER_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" libasmpw$(LIBEXT)

$(CMSIS_DSP_LIB):
	$(Q) $(MAKE) -C $(CMSIS_DSP_DIR) TOPDIR="$(TOPDIR)" SDKDIR="$(SDKDIR)" APPDIR="$(APPDIR)"

context:

-include Make.dep
