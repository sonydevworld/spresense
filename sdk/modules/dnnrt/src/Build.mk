############################################################################
# modules/dnnrt/src/Makefile
#
#   Copyright 2018 Sony Corporation
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
# 3. Neither the name of Sony Corporation nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
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

-include $(TOPDIR)/.config
-include $(TOPDIR)/Make.defs
-include $(SDKDIR)/Make.defs

DELIM ?= $(strip /)
RUNTIME_EXTERN_DIR := $(SDKDIR)/../externals/nnabla-c-runtime


RUNTIME_EXTERN_SRCDIR := $(RUNTIME_EXTERN_DIR)/src

INCLUDES := -I$(RUNTIME_EXTERN_DIR)/include
INCLUDES += -I$(RUNTIME_EXTERN_SRCDIR)/runtime
INCLUDES += -I$(RUNTIME_EXTERN_SRCDIR)/functions
INCLUDES += -Isrc

CSRCS +=  runtime_nnabla.c
CSRCS +=  shared_chunk.c
CSRCS +=  affine.c
CSRCS +=  convolution.c
CSRC_PATH += src/functions
CSRC_PATH += src/runtime

VPATH += $(CSRC_PATH)
ROOTDEPPATH =$(foreach dir,$(CSRC_PATH), --dep-path $(dir))

ifeq ($(WINTOOL),y)
  INCLUDES += -I"${shell cygpath -w $(SDKDIR)/modules/include/dnnrt}"
  INCLUDES += -I"${shell cygpath -w $(SDKDIR)/modules/include}"
else
  INCLUDES += -I$(SDKDIR)/modules/include/dnnrt
  INCLUDES += -I$(SDKDIR)/modules/include
endif # ($(WINTOOL),y)

CFLAGS += $(INCLUDES) -std=c99
CFLAGS := $(patsubst -O%,-O3,$(CFLAGS)) # don't follow the -O. option in CFLAGS

CMSIS_CFLAGS =  -I "$(SDKDIR)/../externals/cmsis/CMSIS_5/CMSIS/Core/Include" \
				-I "$(SDKDIR)/../externals/cmsis/CMSIS_5/CMSIS/DSP/Include" \
				-I "$(SDKDIR)/../externals/cmsis/CMSIS_5/CMSIS/NN/Include" \
				-D__FPU_PRESENT=1U \
				-DARM_MATH_CM4

CFLAGS += $(CMSIS_CFLAGS)

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

LIB = libdnnrt$(LIBEXT)

#Set NNABLA CMAKE VAR
NNABLA_C_RUNTIME_MAKEFILE = $(RUNTIME_EXTERN_DIR)/build/Makefile
NNABLA_CMAKE_TXT = $(RUNTIME_EXTERN_DIR)/CMakeLists.txt
NNABLA_CMAKE_TOOLCHAIN_FILE = $(SDKDIR)/modules/dnnrt/nnabla-c-runtime.cmake
NNABLA_CMAKE_LINKER = $(SDKDIR)/modules/dnnrt/dummy_linker.sh
NNABLA_LIBS = libnnablart_runtime.a libnnablart_functions.a
NNABLA_CMAKE_PATH = $(RUNTIME_EXTERN_DIR)/build
NNABLA_CMAKE_FORCE_C_COMPILER = $(shell which $(CC))
NNABLA_CMAKE_FORCE_CXX_COMPILER = $(shell which $(CXX))
NNABLA_CMAKE_C_FLAGS = "$(CFLAGS)"
NNABLA_CMAKE_AR = $(word 1, $(AR))

# Common build
.PHONY: context depend clean distclean libmakedep preconfig dnnrt-auto-format

all: .built

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(MAINOBJ): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(OBJS): Makefile

$(LIB): $(OBJS)
	$(call ARCHIVE, $(LIB), $(OBJS))

.built: $(NNABLA_LIBS) $(LIB)
	$(Q) touch .built

install:

context:
# make apps refer to the same version of network.h
	$(Q) install -m 0664 \
		$(RUNTIME_EXTERN_DIR)/include/nnablart/network.h \
		$(SDKDIR)/modules/include/dnnrt/nnablart/network.h

#Compile NNABLA_C_RUNTIME
$(NNABLA_C_RUNTIME_MAKEFILE):$(NNABLA_CMAKE_TXT)
	@if [ -d $(NNABLA_CMAKE_PATH) ]; then rm -rf $(NNABLA_CMAKE_PATH); fi
	@mkdir -p $(NNABLA_CMAKE_PATH)
	cd $(NNABLA_CMAKE_PATH) && cmake -DARM_CC_PATH=$(NNABLA_CMAKE_FORCE_C_COMPILER) \
	 -DARM_CXX_PATH=$(NNABLA_CMAKE_FORCE_CXX_COMPILER) \
	 -DCMAKE_CFLAGS=$(NNABLA_CMAKE_C_FLAGS) \
	 -DCMAKE_AR_TOOL=$(NNABLA_CMAKE_AR) \
	 -DCMAKE_LINKER=$(NNABLA_CMAKE_LINKER) \
	 -DCMAKE_TOOLCHAIN_FILE=$(NNABLA_CMAKE_TOOLCHAIN_FILE) ..

$(NNABLA_LIBS):$(NNABLA_C_RUNTIME_MAKEFILE)
	@cd $(NNABLA_CMAKE_PATH) && make
	@mv $(NNABLA_CMAKE_PATH)/src/functions/libnnablart_functions.a .
	@mv $(NNABLA_CMAKE_PATH)/src/runtime/libnnablart_runtime.a .

# Create dependencies

libmakedep: $(SDKDIR)/.config Makefile
	$(Q) $(MKDEP) $(ROOTDEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep

.depend: libmakedep 
	$(Q) touch $@

depend: .depend

clean:
	$(call CLEAN)
	$(call DELFILE, .built)
	$(call DELFILE, Make.dep)
	$(call DELFILE, *.o)
	$(call DELFILE, *.d)
	$(call DELFILE, $(LIB))
	$(Q) rm -rf $(NNABLA_CMAKE_PATH)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)
	$(call DELFILE, $(SDKDIR)/modules/include/dnnrt/nnablart/network.h)

-include Make.dep
preconfig:

dnnrt-auto-format:
	@find src/runtime/ -type f -name "*.[ch]" |xargs -n1 clang-format -i
	@find src/functions/ -type f -name "*.[ch]" |xargs -n1 clang-format -i
