############################################################################
# modules/tflmrt/src-mpcomm/worker/tflmrt-mp/Makefile
#
#   Copyright 2022 Sony Semiconductor Solutions Corporation
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

BIN = TFLMRT-MP

CSRCS += tflm_main.c

CSRCS += tflm_controller.c

ASMPW_DIR = "../lib/asmpw"
MPCOMMW_DIR = "../lib/mpcommw"
TENSORFLOW_MICROLITE_DIR = "../lib/tensorflow-microlite"

LDLIBPATH += -L $(ASMPW_DIR)
LDLIBPATH += -L $(MPCOMMW_DIR)
LDLIBPATH += -L $(TENSORFLOW_MICROLITE_DIR)

LDLIBS += -lmpcommw
LDLIBS += -lasmpw
LDLIBS += -ltensorflow-microlite

LIBM = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-file-name=libm.a}"
LIBC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-file-name=libc_nano.a}"
LIBGCC = "$(shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name)"

TENSORFLOW_RUNTIME_DIR := $(SDKDIR)/../externals/tensorflow/c-runtime

CELFFLAGS += -I"$(TENSORFLOW_RUNTIME_DIR)"

STACK_SIZE = 8192
WORKERSTACKSIZE = $(STACK_SIZE)
CELFFLAGS += -DSTACK_SIZE=$(STACK_SIZE)
CELFFLAGS += -DTF_LITE_STATIC_MEMORY

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

OBJEXT ?= .o

all: $(BIN)

$(COBJS): %$(OBJEXT): %.c
	@echo "CC: $<"
	$(Q) $(CC) -c $(CELFFLAGS) $< -o $@

$(AOBJS): %$(OBJEXT): %.S
	@echo "AS: $<"
	$(Q) $(CC) -c $(AFLAGS) $< -o $@

$(BIN): $(COBJS) $(AOBJS)
	@echo "LD: $<"
	$(Q) $(LD) $(LDRAWELFFLAGS) $(LDLIBPATH) -o $@ $(ARCHCRT0OBJ) $^ $(LDLIBS) $(LIBM) $(LIBC) $(LIBGCC)
	$(Q) $(STRIP) -d $(BIN)

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)
