############################################################################
# modules/dnnrt/src-mpcomm/worker/dnnrt-mp/Makefile
#
#   Copyright 2021 Sony Semiconductor Solutions Corporation
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

BIN = DNNRT-MP

CSRCS += dnn_main.c

CSRCS += dnn_controller.c dnn_helper.c dnn_variable_buffer.c dnn_scratch_buffer.c dnn_exec_function.c

VPATH += functions
CSRCS += affine.c convolution.c

ASMPW_DIR = "../lib/asmpw"
MPCOMMW_DIR = "../lib/mpcommw"
CMSIS_NN_DIR = "../lib/cmsis-nn"
NNABLART_DIR = "../lib/nnablart"

LDLIBPATH += -L $(ASMPW_DIR)
LDLIBPATH += -L $(MPCOMMW_DIR)
LDLIBPATH += -L $(CMSIS_NN_DIR)
LDLIBPATH += -L $(NNABLART_DIR)

LDLIBS += -lmpcommw
LDLIBS += -lasmpw
LDLIBS += -lcmsis_nn
LDLIBS += -lnnablart

LIBM = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-file-name=libm.a}"
LIBC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-file-name=libc_nano.a}"
LIBGCC = "$(shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name)"
LIBNOSYS = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-file-name=libnosys.a}"

CMSIS_DIR := $(SDKDIR)/../externals/cmsis/CMSIS_5
NNABLA_RUNTIME_DIR := $(SDKDIR)/../externals/nnablart/nnabla-c-runtime

CELFFLAGS += -I"$(NNABLA_RUNTIME_DIR)/include"
CELFFLAGS += -I"$(NNABLA_RUNTIME_DIR)/src/runtime"
CELFFLAGS += -I"$(NNABLA_RUNTIME_DIR)/src/functions"
CELFFLAGS += -I"$(CMSIS_DIR)/CMSIS/Core/Include"
CELFFLAGS += -I"$(CMSIS_DIR)/CMSIS/DSP/Include"
CELFFLAGS += -I"$(CMSIS_DIR)/CMSIS/NN/Include"

CELFFLAGS := $(patsubst -O%,-O3,$(CELFFLAGS)) # don't follow the -O. option in CELFFLAGS

STACK_SIZE = 2048
WORKERSTACKSIZE = $(STACK_SIZE)
CELFFLAGS += -D__FPU_PRESENT=1U -DARM_MATH_CM4
CELFFLAGS += -DSTACK_SIZE=$(STACK_SIZE)
CELFFLAGS += -std=c99

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
	$(Q) $(LD) $(LDRAWELFFLAGS) $(LDLIBPATH) -o $@ $(ARCHCRT0OBJ) $^ $(LDLIBS) $(LIBM) $(LIBC) $(LIBGCC) $(LIBNOSYS)
	$(Q) $(STRIP) -d $(BIN)

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)
